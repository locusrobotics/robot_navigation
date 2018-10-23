# nav_2d_utils Conversions

(note: exact syntax differs from below for conciseness, leaving out `const` and `&`)

## Twist Transformations
| to `nav_2d_msgs` | from `nav_2d_msgs` |
| -- | -- |
| `Twist2D twist3Dto2D(geometry_msgs::Twist)` | `geometry_msgs::Twist twist2Dto3D(Twist2D cmd_vel_2d)`

## Pose Transformations
| to `nav_2d_msgs` | from `nav_2d_msgs` |
| -- | -- |
| `Pose2DStamped poseStampedToPose2D(geometry_msgs::PoseStamped)` | `geometry_msgs::PoseStamped pose2DToPoseStamped(Pose2DStamped)`
||`geometry_msgs::PoseStamped pose2DToPoseStamped(geometry_msgs::Pose2D, std::string, ros::Time)`|
||`geometry_msgs::Pose pose2DToPose(geometry_msgs::Pose2D)`|
| `Pose2DStamped stampedPoseToPose2D(tf::Stamped<tf::Pose>)` | |

## Path Transformations
| to `nav_2d_msgs` | from `nav_2d_msgs` |
| -- | -- |
| `Path2D pathToPath(nav_msgs::Path)` |`nav_msgs::Path pathToPath(Path2D)`
| `Path2D posesToPath2D(std::vector<geometry_msgs::PoseStamped>)` | `nav_msgs::Path poses2DToPath(std::vector<geometry_msgs::Pose2D>, std::string, ros::Time)`

Also, `nav_msgs::Path posesToPath(std::vector<geometry_msgs::PoseStamped>)`

## Info Transformations
| to `nav_2d_msgs` | from `nav_2d_msgs` |
| -- | -- |
|`nav_2d_msgs::NavGridInfo toMsg(nav_grid::NavGridInfo)`|`nav_grid::NavGridInfo fromMsg(nav_2d_msgs::NavGridInfo)`|

| to `nav_grid` info | from `nav_grid` info |
| -- | -- |
|`nav_grid::NavGridInfo infoToInfo(nav_msgs::MapMetaData)` | `nav_msgs::MapMetaData infoToInfo(nav_grid::NavGridInfo)`


## Bounds Transformations
| to `nav_2d_msgs` | from `nav_2d_msgs` |
| -- | -- |
|`nav_2d_msgs::UIntBounds toMsg(nav_core2::UIntBounds)`|`nav_core2::UIntBounds fromMsg(nav_2d_msgs::UIntBounds)`|
