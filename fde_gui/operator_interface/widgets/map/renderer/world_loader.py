
import math
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

import numpy as np
import pyvista as pv


class WorldLoader:
    """Loads static Gazebo SDF visuals with OBJ materials into PyVista."""

    def __init__(self, models_dir):
        self.models_dir = Path(models_dir)
        self.mesh_cache = {}
        self.texture_cache = {}
        self.material_cache = {}

    @staticmethod
    def numbers(text, default):
        if not text:
            return list(default)
        values = [float(v) for v in text.split()]
        return (values + list(default[len(values):]))[:len(default)]

    def pose(self, element):
        pose = element.find("pose")
        return self.numbers(
            pose.text if pose is not None else None,
            [0, 0, 0, 0, 0, 0],
        )

    @staticmethod
    def transform(mesh, pose):
        x, y, z, roll, pitch, yaw = pose
        mesh.rotate_x(math.degrees(roll), inplace=True)
        mesh.rotate_y(math.degrees(pitch), inplace=True)
        mesh.rotate_z(math.degrees(yaw), inplace=True)
        mesh.translate((x, y, z), inplace=True)

    def resolve_uri(self, uri):
        if not uri.startswith("model://"):
            return None
        return self.models_dir / uri[len("model://"):]

    def load_mesh(self, path):
        path = Path(path)
        if path not in self.mesh_cache:
            self.mesh_cache[path] = pv.read(path)
        return self.mesh_cache[path].copy(deep=True)

    def load_texture(self, path):
        path = Path(path)
        if path not in self.texture_cache:
            self.texture_cache[path] = pv.read_texture(path)
        return self.texture_cache[path]

    def load_mtl(self, path):
        """Read diffuse colours and texture paths from an MTL file."""
        path = Path(path)

        if path in self.material_cache:
            return self.material_cache[path]

        materials = {}
        current = None

        if path.is_file():
            for raw in path.read_text(errors="replace").splitlines():
                line = raw.strip()
                if not line or line.startswith("#"):
                    continue

                parts = line.split()
                command = parts[0]

                if command == "newmtl" and len(parts) >= 2:
                    current = " ".join(parts[1:])
                    materials[current] = {}

                elif current is not None:
                    if command == "Kd" and len(parts) >= 4:
                        materials[current]["color"] = tuple(
                            float(v) for v in parts[1:4]
                        )

                    elif command == "map_Kd" and len(parts) >= 2:
                        materials[current]["texture"] = (
                            path.parent / " ".join(parts[1:])
                        )

        self.material_cache[path] = materials
        return materials

    def load_obj_material_groups(self, path):
        """
        Build separate PyVista meshes for each OBJ material.

        Supports OBJ vertex/UV/normal indices, including negative
        vertex indices. UV coordinates are retained per face corner.
        """
        path = Path(path)

        if path in self.mesh_cache:
            return self.mesh_cache[path]

        vertices = []
        texcoords = []
        groups = defaultdict(list)
        materials = {}
        active_material = "__default__"

        for raw in path.read_text(errors="replace").splitlines():
            line = raw.strip()
            if not line or line.startswith("#"):
                continue

            parts = line.split()
            command = parts[0]

            if command == "mtllib":
                for filename in parts[1:]:
                    materials.update(self.load_mtl(path.parent / filename))

            elif command == "v" and len(parts) >= 4:
                vertices.append(tuple(float(v) for v in parts[1:4]))

            elif command == "vt" and len(parts) >= 3:
                texcoords.append(tuple(float(v) for v in parts[1:3]))

            elif command == "usemtl" and len(parts) >= 2:
                active_material = " ".join(parts[1:])

            elif command == "f" and len(parts) >= 4:
                corners = []

                for token in parts[1:]:
                    indices = token.split("/")

                    vi = int(indices[0])
                    vi = vi - 1 if vi > 0 else len(vertices) + vi

                    ti = None
                    if len(indices) > 1 and indices[1]:
                        ti = int(indices[1])
                        ti = ti - 1 if ti > 0 else len(texcoords) + ti

                    corners.append((vi, ti))

                groups[active_material].append(corners)

        result = []

        for material_name, faces in groups.items():
            points = []
            uv_values = []
            cell_faces = []

            # Each face corner receives its own point so UV seams
            # are preserved without requiring vertex deduplication.
            for face in faces:
                start = len(points)

                for vertex_index, texture_index in face:
                    points.append(vertices[vertex_index])
                    if texture_index is not None:
                        uv_values.append(texcoords[texture_index])
                    else:
                        uv_values.append((0.0, 0.0))

                cell_faces.extend(
                    [len(face), *range(start, start + len(face))]
                )

            if not points:
                continue

            mesh = pv.PolyData(points, faces=cell_faces)

            if texcoords:
                mesh.active_texture_coordinates = np.asarray(
                    uv_values,
                    dtype=np.float32,
                )

            result.append((
                mesh,
                materials.get(material_name, {}),
                material_name,
            ))

        self.mesh_cache[path] = result
        return result

    def add_visual_mesh(
        self, plotter, mesh, color, texture=None
    ):
        plotter.add_mesh(
            mesh,
            color=(1.0, 1.0, 1.0) if texture is not None else color,
            texture=texture,
            show_edges=False,
            ambient=0.45,
            diffuse=0.55,
            specular=0.05,
            smooth_shading=False,
        )

    def load_visual(self, plotter, visual, model_pose, link_pose):
        geometry = visual.find("geometry")
        if geometry is None:
            return False

        material = visual.find("material")
        sdf_color = (0.72, 0.80, 0.82)

        if material is not None:
            diffuse = material.findtext("diffuse")
            if diffuse:
                sdf_color = tuple(
                    self.numbers(diffuse, [1, 1, 1, 1])[:3]
                )

        visual_pose = self.pose(visual)
        mesh_def = geometry.find("mesh")

        if mesh_def is not None:
            uri = mesh_def.findtext("uri", "").strip()
            path = self.resolve_uri(uri)

            if path is None or not path.exists():
                print(f"Missing mesh: {uri}")
                return False

            scale = self.numbers(
                mesh_def.findtext("scale"), [1, 1, 1]
            )

            # The snow tree uses an albedo image with OBJ UVs.
            tree_texture = None

            if path.parent.parent.name == "snow_tree":
                texture_path = (
                    self.models_dir
                    / "snow_tree/materials/textures/snow_tree_albedo.png"
                )
                if texture_path.is_file():
                    tree_texture = self.load_texture(texture_path)

            groups = self.load_obj_material_groups(path)

            for original_mesh, obj_material, name in groups:
                mesh = original_mesh.copy(deep=True)

                mesh.scale(scale, inplace=True)
                self.transform(mesh, visual_pose)
                self.transform(mesh, link_pose)
                self.transform(mesh, model_pose)

                color = obj_material.get("color", sdf_color)
                texture = tree_texture

                if texture is None:
                    texture_path = obj_material.get("texture")
                    if texture_path and texture_path.is_file():
                        texture = self.load_texture(texture_path)

                self.add_visual_mesh(
                    plotter, mesh, color, texture
                )

            return bool(groups)

        # Primitive SDF geometry.
        mesh = None

        plane = geometry.find("plane")
        box = geometry.find("box")

        if plane is not None:
            sx, sy = self.numbers(
                plane.findtext("size"), [1, 1]
            )
            mesh = pv.Plane(
                i_size=sx,
                j_size=sy,
                direction=(0, 0, 1),
            )

        elif box is not None:
            sx, sy, sz = self.numbers(
                box.findtext("size"), [1, 1, 1]
            )
            mesh = pv.Box(bounds=(
                -sx / 2, sx / 2,
                -sy / 2, sy / 2,
                -sz / 2, sz / 2,
            ))

        if mesh is None:
            return False

        self.transform(mesh, visual_pose)
        self.transform(mesh, link_pose)
        self.transform(mesh, model_pose)

        self.add_visual_mesh(plotter, mesh, sdf_color)
        return True

    def load_model(self, plotter, model_name, model_pose):
        model_file = self.models_dir / model_name / "model.sdf"

        if not model_file.is_file():
            print(f"Missing model: {model_name}")
            return 0

        root = ET.parse(model_file).getroot()
        model = root.find("model")

        if model is None:
            return 0

        count = 0

        for link in model.findall("link"):
            link_pose = self.pose(link)

            for visual in link.findall("visual"):
                if self.load_visual(
                    plotter, visual, model_pose, link_pose
                ):
                    count += 1

        return count

    def load_world(self, plotter, world_file):
        root = ET.parse(world_file).getroot()
        world = root.find("world")

        if world is None:
            raise ValueError("No world element in SDF")

        total = 0

        for include in world.findall("include"):
            uri = include.findtext("uri", "").strip()

            if not uri.startswith("model://"):
                continue

            model_name = uri[len("model://"):].split("/")[0]

            # Hidden hazards must remain invisible until detected.
            if model_name.startswith("ice_block"):
                continue

            total += self.load_model(
                plotter,
                model_name,
                self.pose(include),
            )

        print(f"Loaded {total} static visual elements")
        return total
