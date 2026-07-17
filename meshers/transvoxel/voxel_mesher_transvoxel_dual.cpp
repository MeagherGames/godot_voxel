#include "voxel_mesher_transvoxel_dual.h"

#include "../../constants/cube_tables.h"
#include "../../thirdparty/meshoptimizer/meshoptimizer.h"
#include "../../storage/voxel_buffer_gd.h"
#include "../../util/godot/classes/material.h"
#include "../../util/godot/classes/mesh.h"
#include "../../util/godot/classes/rendering_server.h"
#include "../../util/godot/core/packed_arrays.h"
#include "../../util/math/conv.h"

using namespace zylann::godot;

namespace zylann::voxel {

namespace {

int normalize_secondary_channel(int channel) {
	switch (channel) {
		case 0:
			return VoxelBuffer::CHANNEL_DATA5;
		case 1:
			return VoxelBuffer::CHANNEL_DATA6;
		case 2:
			return VoxelBuffer::CHANNEL_DATA7;
		default:
			return channel;
	}
}

void copy_channel_remapped(
		const VoxelBuffer &src,
		const unsigned int src_channel,
		VoxelBuffer &dst,
		const unsigned int dst_channel
) {
	dst.set_channel_depth(dst_channel, src.get_channel_depth(src_channel));

	if (src.get_channel_compression(src_channel) == VoxelBuffer::COMPRESSION_UNIFORM) {
		dst.clear_channel(dst_channel, src.get_voxel(0, 0, 0, src_channel));
		return;
	}

	Span<const uint8_t> src_bytes;
	ZN_ASSERT_RETURN(src.get_channel_as_bytes_read_only(src_channel, src_bytes));
	dst.set_channel_from_bytes(dst_channel, src_bytes);
}

void create_secondary_meshing_buffer(
		const VoxelBuffer &source,
		const uint32_t secondary_sdf_channel,
		const VoxelMesherTransvoxel::TexturingMode texturing_mode,
		VoxelBuffer &out_voxels
) {
	out_voxels.create(source.get_size());
	source.copy_to(out_voxels, false);

	copy_channel_remapped(source, secondary_sdf_channel, out_voxels, VoxelBuffer::CHANNEL_SDF);

	switch (texturing_mode) {
		case VoxelMesherTransvoxel::TEXTURES_MIXEL4_S4:
			copy_channel_remapped(source, VoxelBuffer::CHANNEL_DATA6, out_voxels, VoxelBuffer::CHANNEL_INDICES);
			copy_channel_remapped(source, VoxelBuffer::CHANNEL_DATA7, out_voxels, VoxelBuffer::CHANNEL_WEIGHTS);
			break;

		case VoxelMesherTransvoxel::TEXTURES_SINGLE_S4:
			copy_channel_remapped(source, VoxelBuffer::CHANNEL_DATA6, out_voxels, VoxelBuffer::CHANNEL_INDICES);
			break;

		case VoxelMesherTransvoxel::TEXTURES_NONE:
			break;

		default:
			break;
	}
}

void fill_surface_arrays(Array &arrays, const transvoxel::MeshArrays &src, const int texturing_array_index) {
	PackedVector3Array vertices;
	PackedVector3Array normals;
	PackedFloat32Array lod_data;
	PackedFloat32Array texturing_data;
	PackedInt32Array indices;

	copy_to(vertices, to_span_const(src.vertices));

	lod_data.resize(src.lod_data.size() * 4);
	static_assert(sizeof(transvoxel::LodAttrib) == 16);
	memcpy(lod_data.ptrw(), src.lod_data.data(), lod_data.size() * sizeof(float));

	copy_to(indices, to_span_const(src.indices));

	arrays.resize(Mesh::ARRAY_MAX);
	arrays[Mesh::ARRAY_VERTEX] = vertices;
	if (src.normals.size() != 0) {
		copy_to(normals, to_span_const(src.normals));
		arrays[Mesh::ARRAY_NORMAL] = normals;
	}

	if (src.texturing_data_1f32.size() != 0) {
		texturing_data.resize(src.texturing_data_1f32.size());
		memcpy(texturing_data.ptrw(), src.texturing_data_1f32.data(), texturing_data.size() * sizeof(float));
		arrays[texturing_array_index] = texturing_data;
	} else if (src.texturing_data_2f32.size() != 0) {
		texturing_data.resize(src.texturing_data_2f32.size() * 2);
		memcpy(texturing_data.ptrw(), src.texturing_data_2f32.data(), texturing_data.size() * sizeof(float));
		arrays[texturing_array_index] = texturing_data;
	}

	arrays[Mesh::ARRAY_CUSTOM0] = lod_data;
	arrays[Mesh::ARRAY_INDEX] = indices;
}

template <typename T>
void remap_vertex_array(
		const StdVector<T> &src_data,
		StdVector<T> &dst_data,
		const StdVector<unsigned int> &remap_indices,
		const unsigned int unique_vertex_count
) {
	if (src_data.size() == 0) {
		dst_data.clear();
		return;
	}
	dst_data.resize(unique_vertex_count);
	zylannmeshopt::meshopt_remapVertexBuffer(
			&dst_data[0], &src_data[0], src_data.size(), sizeof(T), remap_indices.data()
	);
}

void simplify_mesh(
		const transvoxel::MeshArrays &src_mesh,
		transvoxel::MeshArrays &dst_mesh,
		const float target_ratio,
		const float error_threshold
) {
	ERR_FAIL_COND(target_ratio < 0.f || target_ratio > 1.f);
	ERR_FAIL_COND(error_threshold < 0.f || error_threshold > 1.f);
	ERR_FAIL_COND(src_mesh.vertices.size() < 3);
	ERR_FAIL_COND(src_mesh.indices.size() < 3);

	const unsigned int target_index_count = target_ratio * src_mesh.indices.size();

	StdVector<unsigned int> lod_indices;
	lod_indices.resize(src_mesh.indices.size());

	float lod_error = 0.f;
	const unsigned int lod_index_count = zylannmeshopt::meshopt_simplify(
			&lod_indices[0],
			reinterpret_cast<const unsigned int *>(src_mesh.indices.data()),
			src_mesh.indices.size(),
			&src_mesh.vertices[0].x,
			src_mesh.vertices.size(),
			sizeof(Vector3f),
			target_index_count,
			error_threshold,
			zylannmeshopt::meshopt_SimplifyLockBorder,
			&lod_error
	);
	lod_indices.resize(lod_index_count);

	StdVector<unsigned int> remap_indices;
	remap_indices.resize(src_mesh.vertices.size());

	const unsigned int unique_vertex_count = zylannmeshopt::meshopt_optimizeVertexFetchRemap(
			&remap_indices[0], lod_indices.data(), lod_indices.size(), src_mesh.vertices.size()
	);

	remap_vertex_array(src_mesh.vertices, dst_mesh.vertices, remap_indices, unique_vertex_count);
	remap_vertex_array(src_mesh.normals, dst_mesh.normals, remap_indices, unique_vertex_count);
	remap_vertex_array(src_mesh.lod_data, dst_mesh.lod_data, remap_indices, unique_vertex_count);
	remap_vertex_array(src_mesh.texturing_data_1f32, dst_mesh.texturing_data_1f32, remap_indices, unique_vertex_count);
	remap_vertex_array(src_mesh.texturing_data_2f32, dst_mesh.texturing_data_2f32, remap_indices, unique_vertex_count);

	dst_mesh.indices.resize(lod_indices.size());
	zylannmeshopt::meshopt_remapIndexBuffer(
			reinterpret_cast<unsigned int *>(dst_mesh.indices.data()), lod_indices.data(), lod_indices.size(),
			remap_indices.data()
	);
}

bool build_secondary_surface(
		const VoxelBuffer &voxels,
		const uint32_t sdf_channel,
		const VoxelMesherTransvoxelDual &mesher,
		const VoxelMesher::Input &input,
		VoxelMesher::Output::Surface &surface_out,
		const int texturing_array_index
) {
	if (voxels.is_uniform(sdf_channel)) {
		return false;
	}

	transvoxel::Cache cache;
	transvoxel::MeshArrays mesh_arrays;
	StdVector<transvoxel::CellInfo> cell_infos;

	const transvoxel::TexturingMode texturing_mode = static_cast<transvoxel::TexturingMode>(mesher.get_texturing_mode());
	const transvoxel::DefaultTextureIndicesData default_texture_indices_data = transvoxel::build_regular_mesh(
			voxels,
			sdf_channel,
			input.lod_index,
			texturing_mode,
			cache,
			mesh_arrays,
			input.detail_texture_hint ? &cell_infos : nullptr,
			mesher.get_edge_clamp_margin(),
			mesher.get_textures_ignore_air_voxels()
	);

	if (mesh_arrays.vertices.size() == 0 || mesh_arrays.indices.size() == 0) {
		return false;
	}

	if (mesher.is_mesh_optimization_enabled()) {
		simplify_mesh(
				mesh_arrays,
				mesh_arrays,
				mesher.get_mesh_optimization_target_ratio(),
				mesher.get_mesh_optimization_error_threshold()
		);
	}

	if (mesher.get_transitions_enabled() && input.lod_hint) {
		for (int dir = 0; dir < Cube::SIDE_COUNT; ++dir) {
			transvoxel::build_transition_mesh(
					voxels,
					sdf_channel,
					dir,
					input.lod_index,
					texturing_mode,
					cache,
					mesh_arrays,
					default_texture_indices_data,
					mesher.get_edge_clamp_margin(),
					mesher.get_textures_ignore_air_voxels()
			);
		}
	}

	if (mesh_arrays.vertices.size() == 0 || mesh_arrays.indices.size() == 0) {
		return false;
	}

	Array gd_arrays;
	fill_surface_arrays(gd_arrays, mesh_arrays, texturing_array_index);
	surface_out.arrays = gd_arrays;
	surface_out.material_index = 1;
	return true;
}

} // namespace

VoxelMesherTransvoxelDual::VoxelMesherTransvoxelDual() = default;

VoxelMesherTransvoxelDual::~VoxelMesherTransvoxelDual() = default;

int VoxelMesherTransvoxelDual::get_used_channels_mask() const {
	uint32_t mask = VoxelMesherTransvoxel::get_used_channels_mask() | (1 << _secondary_sdf_channel);

	switch (get_texturing_mode()) {
		case TEXTURES_MIXEL4_S4:
			mask |= (1 << VoxelBuffer::CHANNEL_DATA6) | (1 << VoxelBuffer::CHANNEL_DATA7);
			break;

		case TEXTURES_SINGLE_S4:
			mask |= (1 << VoxelBuffer::CHANNEL_DATA6);
			break;

		case TEXTURES_NONE:
			break;

		default:
			break;
	}

	return mask;
}

Ref<Material> VoxelMesherTransvoxelDual::get_material_by_index(unsigned int i) const {
	switch (i) {
		case 0:
			return _primary_material;
		case 1:
			return _secondary_material;
		default:
			return Ref<Material>();
	}
}

unsigned int VoxelMesherTransvoxelDual::get_material_index_count() const {
	return 2;
}

void VoxelMesherTransvoxelDual::build(VoxelMesher::Output &output, const VoxelMesher::Input &input) {
	const uint32_t secondary_channel = static_cast<uint32_t>(normalize_secondary_channel(_secondary_sdf_channel));
	ERR_FAIL_COND(secondary_channel < VoxelBuffer::CHANNEL_DATA5 || secondary_channel > VoxelBuffer::CHANNEL_DATA7);
	const TexturingMode texturing_mode = get_texturing_mode();

	VoxelBuffer secondary_voxels(VoxelBuffer::ALLOCATOR_POOL);
	create_secondary_meshing_buffer(input.voxels, secondary_channel, texturing_mode, secondary_voxels);

	VoxelMesher::Output::Surface secondary_surface;
	const bool has_secondary =
			build_secondary_surface(secondary_voxels, VoxelBuffer::CHANNEL_SDF, *this, input, secondary_surface, Mesh::ARRAY_CUSTOM2);

	VoxelMesherTransvoxel::build(output, input);

	output.primitive_type = Mesh::PRIMITIVE_TRIANGLES;
	output.mesh_flags |= (RenderingServerEnums::ARRAY_CUSTOM_RGBA_FLOAT << Mesh::ARRAY_FORMAT_CUSTOM0_SHIFT);
	if (texturing_mode == TEXTURES_MIXEL4_S4 || texturing_mode == TEXTURES_SINGLE_S4) {
		output.mesh_flags |= (RenderingServerEnums::ARRAY_CUSTOM_RG_FLOAT << Mesh::ARRAY_FORMAT_CUSTOM1_SHIFT);
		output.mesh_flags |= (RenderingServerEnums::ARRAY_CUSTOM_RG_FLOAT << Mesh::ARRAY_FORMAT_CUSTOM2_SHIFT);
	}

	if (has_secondary) {
		secondary_surface.collision_enabled = _generate_secondary_collision;
		output.surfaces.push_back(std::move(secondary_surface));
	}
}

void VoxelMesherTransvoxelDual::set_secondary_sdf_channel(int channel) {
	channel = normalize_secondary_channel(channel);
	ERR_FAIL_COND(channel < VoxelBuffer::CHANNEL_DATA5 || channel > VoxelBuffer::CHANNEL_DATA7);
	_secondary_sdf_channel = channel;
	emit_changed();
}

int VoxelMesherTransvoxelDual::get_secondary_sdf_channel() const {
	return _secondary_sdf_channel;
}

void VoxelMesherTransvoxelDual::set_generate_secondary_collision(bool enabled) {
	_generate_secondary_collision = enabled;
	emit_changed();
}

bool VoxelMesherTransvoxelDual::is_generating_secondary_collision() const {
	return _generate_secondary_collision;
}

void VoxelMesherTransvoxelDual::set_primary_material(Ref<Material> material) {
	_primary_material = material;
	emit_changed();
}

Ref<Material> VoxelMesherTransvoxelDual::get_primary_material() const {
	return _primary_material;
}

void VoxelMesherTransvoxelDual::set_secondary_material(Ref<Material> material) {
	_secondary_material = material;
	emit_changed();
}

Ref<Material> VoxelMesherTransvoxelDual::get_secondary_material() const {
	return _secondary_material;
}

void VoxelMesherTransvoxelDual::_bind_methods() {
	using Self = VoxelMesherTransvoxelDual;

	ClassDB::bind_method(D_METHOD("set_secondary_sdf_channel", "channel"), &Self::set_secondary_sdf_channel);
	ClassDB::bind_method(D_METHOD("get_secondary_sdf_channel"), &Self::get_secondary_sdf_channel);
	ClassDB::bind_method(
			D_METHOD("set_generate_secondary_collision", "enabled"),
			&Self::set_generate_secondary_collision
	);
	ClassDB::bind_method(D_METHOD("is_generating_secondary_collision"), &Self::is_generating_secondary_collision);
	ClassDB::bind_method(D_METHOD("set_primary_material", "material"), &Self::set_primary_material);
	ClassDB::bind_method(D_METHOD("get_primary_material"), &Self::get_primary_material);
	ClassDB::bind_method(D_METHOD("set_secondary_material", "material"), &Self::set_secondary_material);
	ClassDB::bind_method(D_METHOD("get_secondary_material"), &Self::get_secondary_material);

	ADD_PROPERTY(
			PropertyInfo(
					Variant::INT,
					"secondary_sdf_channel",
					PROPERTY_HINT_ENUM,
					zylann::voxel::godot::VoxelBuffer::CHANNEL_ID_HINT_STRING
			),
			"set_secondary_sdf_channel",
			"get_secondary_sdf_channel"
	);
	ADD_PROPERTY(
			PropertyInfo(Variant::BOOL, "generate_secondary_collision"),
			"set_generate_secondary_collision",
			"is_generating_secondary_collision"
	);
	ADD_PROPERTY(
			PropertyInfo(Variant::OBJECT, "primary_material", PROPERTY_HINT_RESOURCE_TYPE, "Material"),
			"set_primary_material",
			"get_primary_material"
	);
	ADD_PROPERTY(
			PropertyInfo(Variant::OBJECT, "secondary_material", PROPERTY_HINT_RESOURCE_TYPE, "Material"),
			"set_secondary_material",
			"get_secondary_material"
	);
}

} // namespace zylann::voxel