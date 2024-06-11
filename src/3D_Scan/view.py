import open3d as o3d


pcd = o3d.io.read_point_cloud("/home/fsociety/open3d_data/extract/LoungeRGBDImages/scene/integrated.ply")
o3d.visualization.draw_geometries([pcd])
