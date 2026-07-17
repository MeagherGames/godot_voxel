#ifndef VOXEL_MESHER_TRANSVOXEL_DUAL_H
#define VOXEL_MESHER_TRANSVOXEL_DUAL_H

#include "voxel_mesher_transvoxel.h"

namespace zylann::voxel {

class VoxelMesherTransvoxelDual : public VoxelMesherTransvoxel {
	GDCLASS(VoxelMesherTransvoxelDual, VoxelMesherTransvoxel)

public:
	VoxelMesherTransvoxelDual();
	~VoxelMesherTransvoxelDual();

	void build(VoxelMesher::Output &output, const VoxelMesher::Input &input) override;
	int get_used_channels_mask() const override;

	void set_secondary_sdf_channel(int channel);
	int get_secondary_sdf_channel() const;
	void set_generate_secondary_collision(bool enabled);
	bool is_generating_secondary_collision() const;

protected:
	static void _bind_methods();

private:
	int _secondary_sdf_channel = VoxelBuffer::CHANNEL_DATA5;
	bool _generate_secondary_collision = false;
};

} // namespace zylann::voxel

#endif // VOXEL_MESHER_TRANSVOXEL_DUAL_H