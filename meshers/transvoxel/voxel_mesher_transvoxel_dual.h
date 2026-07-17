#ifndef VOXEL_MESHER_TRANSVOXEL_DUAL_H
#define VOXEL_MESHER_TRANSVOXEL_DUAL_H

#include "voxel_mesher_transvoxel.h"

ZN_GODOT_FORWARD_DECLARE(class Material)

namespace zylann::voxel {

class VoxelMesherTransvoxelDual : public VoxelMesherTransvoxel {
	GDCLASS(VoxelMesherTransvoxelDual, VoxelMesherTransvoxel)

public:
	VoxelMesherTransvoxelDual();
	~VoxelMesherTransvoxelDual();

	void build(VoxelMesher::Output &output, const VoxelMesher::Input &input) override;
	int get_used_channels_mask() const override;
	Ref<Material> get_material_by_index(unsigned int i) const override;
	unsigned int get_material_index_count() const override;

	void set_secondary_sdf_channel(int channel);
	int get_secondary_sdf_channel() const;
	void set_generate_secondary_collision(bool enabled);
	bool is_generating_secondary_collision() const;
	void set_primary_material(Ref<Material> material);
	Ref<Material> get_primary_material() const;
	void set_secondary_material(Ref<Material> material);
	Ref<Material> get_secondary_material() const;

protected:
	static void _bind_methods();

private:
	int _secondary_sdf_channel = VoxelBuffer::CHANNEL_DATA5;
	bool _generate_secondary_collision = false;
	Ref<Material> _primary_material;
	Ref<Material> _secondary_material;
};

} // namespace zylann::voxel

#endif // VOXEL_MESHER_TRANSVOXEL_DUAL_H