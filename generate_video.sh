cargo run -r
cd media
rm output.mp4
ffmpeg -framerate 30 -i soft_body_render_%d.png -vcodec libx264 -r 30 -pix_fmt yuv420p output.mp4
cd ../