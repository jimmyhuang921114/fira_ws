import pyrealsense2 as rs

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

profile = pipeline.start(config)
color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
intrinsics = color_stream.get_intrinsics()

print(f"Width: {intrinsics.width}")
print(f"Height: {intrinsics.height}")
print(f"Fx: {intrinsics.fx}")
print(f"Fy: {intrinsics.fy}")
print(f"Cx: {intrinsics.ppx}")
print(f"Cy: {intrinsics.ppy}")
print(f"Distortion: {intrinsics.model}")
print(f"Coeffs: {intrinsics.coeffs}")

pipeline.stop()
