import open3d as o3d
import os

# Đường dẫn tới file binary gốc
input_file = "lego_loam_map_flat.pcd"  # ← Đảm bảo đường dẫn chính xác
output_file = "converted_ascii.pcd"

# Kiểm tra xem file gốc có tồn tại không
if not os.path.exists(input_file):
    print(f"❌ Lỗi: File {input_file} không tồn tại!")
else:
    # Đọc file .pcd binary
    pcd = o3d.io.read_point_cloud(input_file)

    # Kiểm tra xem point cloud có dữ liệu hay không
    if pcd.is_empty():
        print("❌ Lỗi: Không thể đọc được dữ liệu từ file .pcd")
    else:
        # Ghi lại dưới dạng ASCII
        success = o3d.io.write_point_cloud(output_file, pcd, write_ascii=True)
        if success:
            print(f"✅ Đã convert xong, bạn có thể dùng file: {output_file}")
        else:
            print("❌ Lỗi khi ghi file. Không thể ghi file ASCII.")
