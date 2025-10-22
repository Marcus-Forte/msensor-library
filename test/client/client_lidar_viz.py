import open3d as o3d
import numpy as np
import grpc

# `pip install open3d` required!

from proto_gen import  sensors_pb2_grpc
from google.protobuf.empty_pb2 import Empty

def main():
    with grpc.insecure_channel('192.168.3.232:50051') as channel:
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        opt = vis.get_render_option()
        ctr = vis.get_view_control()
        print("Field of view (before changing) %.2f" % ctr.get_field_of_view())
        ctr.change_field_of_view(step=1.5)
        
        opt.point_size = 5.0  # You can change 5.0 to any pixel size
        opt.background_color = np.array([0, 0, 0])
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.random.rand(200, 3))
        # add a coordinate frame to visualize axes (adjust size as needed)
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0.0, 0.0, 0.0])
        vis.add_geometry(coord_frame)
        vis.add_geometry(pcd)

        stub = sensors_pb2_grpc.SensorServiceStub(channel)
        request = Empty()
        stream = stub.getScan(request)
        for scan in stream:
            print(f"Got points: {len(scan.points)} at {scan.timestamp}")
            
            # 1. Create the (N, 3) NumPy array for XYZ coordinates
            xyz_data = np.array([[p.x, p.y, p.z] for p in scan.points])

            # 2. Create the Open3D PointCloud object
            pcd.points = o3d.utility.Vector3dVector(xyz_data)
            print(xyz_data)
            vis.update_geometry(pcd)
            running = vis.poll_events()
            vis.update_renderer()

if __name__ == "__main__":
    main()
