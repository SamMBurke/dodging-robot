uint32 id

geometry_msgs/Point center
geometry_msgs/Vector3 velocity

float32 heading

float32 length
float32 width

geometry_msgs/Point[] corners

int32 age # scan callbacks since track creation
int32 hits # total frames this track has been matched