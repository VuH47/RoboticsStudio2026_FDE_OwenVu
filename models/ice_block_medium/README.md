# ice_block_medium

Static ice sleeve for alpine RGB tests. This is the Phase-3 demo size.

- **URI:** `model://ice_block_medium`
- **Geometry:** lumpy ice mesh along local **+X** (length 1.00 m, radius ~0.32 m). Collision is still a cylinder of those sizes.
- **Cable:** sits around a 0.34 m diameter cable (radius 0.17 m). Origin is the geometric centre so `set_pose` places the sleeve on the cable centreline.
- **Intended placement:** centre of `span_01` (`t = 0.5`), aligned along the cable (local +X). World include/pose is owned by another agent — this folder is the model only.

Pale blue-white, slightly specular ice is chosen to contrast steel cables (~0.32 0.33 0.35). Shadows off. Collision `mu` 0.4.
