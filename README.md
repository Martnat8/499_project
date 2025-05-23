# project_interfaces

The **project_interfaces** package defines custom ROS 2 service interfaces shared across the system:

- **ChangeStateSrv**  
  Used by the lifecycle coordinator to request lifecycle transitions on one or more nodes

- **SaveSrv**  
  Used by the ring buffer recorder to trigger dumping its in-memory video buffer to disk.


