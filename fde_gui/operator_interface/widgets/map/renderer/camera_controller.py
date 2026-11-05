
import vtk


class MapInteractorStyle(vtk.vtkInteractorStyleUser):
    """Disable VTK's default camera rotation and orbit controls."""

    def __init__(self):
        super().__init__()


class BoundedTopDownCamera:
    """
    Orthographic top-down camera with bounded pan and zoom.

    Orientation:
        Screen up    = World +X
        Screen right = World +Y

    Controls:
        Left drag    = Pan
        Mouse wheel  = Zoom
        R            = Reset (handled by the parent widget)
    """

    def __init__(
        self,
        plotter,
        world_bounds=(-30.0, 90.0, -55.0, 55.0),
        max_zoom=8.0,
    ):
        self.plotter = plotter

        self.x_min, self.x_max, self.y_min, self.y_max = world_bounds

        self.world_width = self.x_max - self.x_min
        self.world_height = self.y_max - self.y_min

        self.world_cx = (self.x_min + self.x_max) / 2.0
        self.world_cy = (self.y_min + self.y_max) / 2.0

        self.max_zoom = max_zoom

        self.dragging = False
        self.last_mouse = None

        self.full_view_scale = None
        self.minimum_scale = None

        self._setup_camera()
        self._setup_events()

    def _viewport_aspect(self):
        width, height = self.plotter.window_size

        if width <= 0 or height <= 0:
            return 1.0

        return width / height

    def _calculate_full_view_scale(self):
        """
        Fit the complete world into the rotated viewport.

        Screen vertical   = World X
        Screen horizontal = World Y
        """
        aspect = self._viewport_aspect()

        scale_for_height = self.world_width / 2.0
        scale_for_width = self.world_height / (2.0 * aspect)

        return max(scale_for_height, scale_for_width)

    def _setup_camera(self):
        camera = self.plotter.camera

        camera.position = (
            self.world_cx,
            self.world_cy,
            150.0,
        )

        camera.focal_point = (
            self.world_cx,
            self.world_cy,
            0.0,
        )

        # The final orientation selected during testing.
        camera.up = (1.0, 0.0, 0.0)

        self.plotter.enable_parallel_projection()

        self.reset_view()

    def reset_view(self):
        self.full_view_scale = self._calculate_full_view_scale()

        self.minimum_scale = self.full_view_scale / self.max_zoom

        camera = self.plotter.camera

        camera.parallel_scale = self.full_view_scale

        camera.position = (
            self.world_cx,
            self.world_cy,
            150.0,
        )

        camera.focal_point = (
            self.world_cx,
            self.world_cy,
            0.0,
        )

        camera.up = (1.0, 0.0, 0.0)

        self.plotter.reset_camera_clipping_range()
        self.plotter.render()

    def _visible_size(self):
        scale = self.plotter.camera.parallel_scale
        aspect = self._viewport_aspect()

        visible_height = scale * 2.0
        visible_width = visible_height * aspect

        return visible_width, visible_height

    def _clamp_center(self, x, y):
        visible_width, visible_height = self._visible_size()

        # The camera is rotated 90 degrees.
        # Screen height corresponds to world X.
        # Screen width corresponds to world Y.
        half_x = visible_height / 2.0
        half_y = visible_width / 2.0

        if visible_height >= self.world_width:
            x = self.world_cx
        else:
            x = max(
                self.x_min + half_x,
                min(self.x_max - half_x, x),
            )

        if visible_width >= self.world_height:
            y = self.world_cy
        else:
            y = max(
                self.y_min + half_y,
                min(self.y_max - half_y, y),
            )

        return x, y

    def _set_center(self, x, y):
        x, y = self._clamp_center(x, y)

        camera = self.plotter.camera

        camera.position = (x, y, 150.0)
        camera.focal_point = (x, y, 0.0)
        camera.up = (1.0, 0.0, 0.0)

        self.plotter.reset_camera_clipping_range()
        self.plotter.render()

    def _mouse_move(self, obj, event):
        if not self.dragging or self.last_mouse is None:
            return

        current = obj.GetEventPosition()

        dx = current[0] - self.last_mouse[0]
        dy = current[1] - self.last_mouse[1]

        self.last_mouse = current

        width, height = self.plotter.window_size

        if width <= 0 or height <= 0:
            return

        visible_width, visible_height = self._visible_size()

        world_y_per_pixel = visible_width / width
        world_x_per_pixel = visible_height / height

        cx, cy, _ = self.plotter.camera.focal_point

        # Final pan directions confirmed during testing.
        new_y = cy + dx * world_y_per_pixel
        new_x = cx - dy * world_x_per_pixel

        self._set_center(new_x, new_y)

    def _left_press(self, obj, event):
        self.dragging = True
        self.last_mouse = obj.GetEventPosition()

    def _left_release(self, obj, event):
        self.dragging = False
        self.last_mouse = None

    def _zoom(self, factor):
        camera = self.plotter.camera

        new_scale = camera.parallel_scale * factor

        new_scale = max(
            self.minimum_scale,
            min(self.full_view_scale, new_scale),
        )

        camera.parallel_scale = new_scale

        cx, cy, _ = camera.focal_point
        self._set_center(cx, cy)

    def _wheel_forward(self, obj, event):
        self._zoom(0.85)

    def _wheel_backward(self, obj, event):
        self._zoom(1.0 / 0.85)

    def _setup_events(self):
        interactor = self.plotter.iren.interactor

        # Replace the default VTK camera interaction style.
        self.interaction_style = MapInteractorStyle()
        interactor.SetInteractorStyle(self.interaction_style)

        interactor.AddObserver(
            "LeftButtonPressEvent",
            self._left_press,
        )

        interactor.AddObserver(
            "LeftButtonReleaseEvent",
            self._left_release,
        )

        interactor.AddObserver(
            "MouseMoveEvent",
            self._mouse_move,
        )

        interactor.AddObserver(
            "MouseWheelForwardEvent",
            self._wheel_forward,
        )

        interactor.AddObserver(
            "MouseWheelBackwardEvent",
            self._wheel_backward,
        )
