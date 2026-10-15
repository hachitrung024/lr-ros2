# Exact source geometry for one evaluation cycle. Parallel arrays map each
# canonical step_id to an index in poses; source poses retain their timestamps.
std_msgs/Header header
uint64 trajectory_id
geometry_msgs/PoseStamped[] poses
uint32[] step_ids
uint32[] source_pose_indices
