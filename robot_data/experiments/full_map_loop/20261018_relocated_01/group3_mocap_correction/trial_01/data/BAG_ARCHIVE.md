# Group 3 ROS bag archive

On 2026-10-21, the closed raw recordings in `bag/` and `bag_continued/`
were losslessly archived as `*.db3.zst` to save local disk space. Each
archive was integrity-tested with `zstd --test` before its corresponding
uncompressed `.db3` file was removed.

The original `metadata.yaml` files are retained. ROS 2 cannot play the
compressed archive files directly. Before replaying either recording,
restore its database segments in that directory:

```bash
zstd -d bag/*.db3.zst
zstd -d bag_continued/*.db3.zst
```

Then use the existing `metadata.yaml` with `ros2 bag play` as usual. The
small `bag_finish/` and `bag_final_paths/` recordings remain directly
playable and were not archived.
