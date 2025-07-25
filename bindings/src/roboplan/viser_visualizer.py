"""
TEMPORARY FILE: Can be removed when this PR is landed and released.
https://github.com/stack-of-tasks/pinocchio/pull/2718
"""

import time

try:
    import hppfcl
except ImportError:
    raise ImportError("hppfcl not found, but it is currently required by this viewer.")

import numpy as np

from pinocchio import pinocchio_pywrap_default as pin
from pinocchio.visualize import BaseVisualizer

try:
    import trimesh  # Required by viser
    import viser
except ImportError:
    import_viser_succeed = False
else:
    import_viser_succeed = True

from roboplan import Scene, RRT, computeFramePath, JointPath


MESH_TYPES = (hppfcl.BVHModelBase, hppfcl.HeightFieldOBBRSS, hppfcl.HeightFieldAABB)


class ViserVisualizer(BaseVisualizer):
    """A Pinocchio visualizer using Viser."""

    def __init__(
        self,
        model=pin.Model(),
        collision_model=None,
        visual_model=None,
        copy_models=False,
        data=None,
        collision_data=None,
        visual_data=None,
    ):
        if not import_viser_succeed:
            msg = (
                "Error while importing the viewer client.\n"
                "Check whether viser is properly installed "
                "(pip install --user viser)."
            )
            raise ImportError(msg)

        super().__init__(
            model,
            collision_model,
            visual_model,
            copy_models,
            data,
            collision_data,
            visual_data,
        )
        self.static_objects = []

    def initViewer(
        self,
        viewer=None,
        open=False,
        loadModel=False,
        host="localhost",
        port="8000",
    ):
        """
        Start a new Viser server and client.
        Note: the server can also be passed in through the `viewer` argument.
        Parameters:
            viewer: An existing ViserServer instance, or None.
                If None, creates a new ViserServer in this visualizer.
            open: If True, automatically opens a browser tab to the visualizer.
            loadModel: If True, loads the Pinocchio models passed in.
            host: If `viewer` is None, this will be the host URL.
            port: If `viewer` is None, this will be the host port.
        """
        if (viewer is not None) and (not isinstance(viewer, viser.ViserServer)):
            raise RuntimeError(
                "'viewer' argument must be None or a valid ViserServer instance."
            )

        self.viewer = viewer or viser.ViserServer(host=host, server_port=port)
        self.frames = {}

        if open:
            import webbrowser

            webbrowser.open(f"http://{self.viewer.get_host()}:{self.viewer.get_port()}")

            # Wait until clients are reported.
            # Otherwise, capturing an image too soon after opening a browser window
            # may not register any clients.
            while len(self.viewer.get_clients()) == 0:
                time.sleep(0.1)

        if loadModel:
            self.loadViewerModel()

    def loadViewerModel(
        self,
        rootNodeName="pinocchio",
        collision_color=None,
        visual_color=None,
        frame_axis_length=0.2,
        frame_axis_radius=0.01,
    ):
        """Load the robot in a Viser viewer.
        Parameters:
            rootNodeName: name to give to the robot in the viewer
            collision_color: optional, color to give to the collision model of
                the robot. Format is a list of four RGBA floating-point numbers
                (between 0 and 1)
            visual_color: optional, color to give to the visual model of
                the robot. Format is a list of four RGBA floating-point numbers
                (between 0 and 1)
            frame_axis_length: optional, length of frame axes if displaying frames.
            frame_axis_radius: optional, radius of frame axes if displaying frames.
        """
        self.viewerRootNodeName = rootNodeName

        # Create root frames to help toggle visibility in the Viser UI.
        self.visualRootNodeName = rootNodeName + "/visual"
        self.visualRootFrame = self.viewer.scene.add_frame(
            self.visualRootNodeName, show_axes=False
        )
        self.collisionRootNodeName = rootNodeName + "/collision"
        self.collisionRootFrame = self.viewer.scene.add_frame(
            self.collisionRootNodeName, show_axes=False
        )
        self.framesRootNodeName = rootNodeName + "/frames"
        self.framesRootFrame = self.viewer.scene.add_frame(
            self.framesRootNodeName, show_axes=False
        )

        # Load visual model
        if (visual_color is not None) and (len(visual_color) != 4):
            raise RuntimeError("visual_color must have 4 elements for RGBA.")
        if self.visual_model is not None:
            for visual in self.visual_model.geometryObjects:
                self.loadViewerGeometryObject(
                    visual, self.visualRootNodeName, visual_color
                )
        self.displayVisuals(True)

        # Load collision model
        if (collision_color is not None) and (len(collision_color) != 4):
            raise RuntimeError("collision_color must have 4 elements for RGBA.")
        if self.collision_model is not None:
            for collision in self.collision_model.geometryObjects:
                self.loadViewerGeometryObject(
                    collision, self.collisionRootNodeName, collision_color
                )
        self.displayCollisions(False)

        # Load frames
        for frame in self.model.frames:
            frame_name = self.framesRootNodeName + "/" + frame.name
            self.frames[frame_name] = self.viewer.scene.add_frame(
                frame_name,
                show_axes=True,
                axes_length=frame_axis_length,
                axes_radius=frame_axis_radius,
            )
        self.displayFrames(False)

    def loadViewerGeometryObject(self, geometry_object, prefix="", color=None):
        """Loads a single geometry object."""
        name = geometry_object.name
        if prefix:
            name = prefix + "/" + name
        geom = geometry_object.geometry
        color_override = color or geometry_object.meshColor

        if isinstance(geom, hppfcl.Box):
            frame = self.viewer.scene.add_box(
                name,
                dimensions=geom.halfSide * 2.0,
                color=color_override[:3],
                opacity=color_override[3],
            )
        elif isinstance(geom, hppfcl.Sphere):
            frame = self.viewer.scene.add_icosphere(
                name,
                radius=geom.radius,
                color=color_override[:3],
                opacity=color_override[3],
            )
        elif isinstance(geom, hppfcl.Cylinder):
            mesh = trimesh.creation.cylinder(
                radius=geom.radius,
                height=geom.halfLength * 2.0,
            )
            frame = self.viewer.scene.add_mesh_simple(
                name,
                mesh.vertices,
                mesh.faces,
                color=color_override[:3],
                opacity=color_override[3],
            )
        elif isinstance(geom, MESH_TYPES):
            mesh = trimesh.load(geometry_object.meshPath)
            if color is None:
                frame = self.viewer.scene.add_mesh_trimesh(name, mesh)
            else:
                frame = self.viewer.scene.add_mesh_simple(
                    name,
                    mesh.vertices,
                    mesh.faces,
                    color=color_override[:3],
                    opacity=color_override[3],
                )
        else:
            raise RuntimeError(f"Unsupported geometry type for {name}: {type(geom)}")

        self.frames[name] = frame

    def display(self, q=None):
        """
        Display the robot at configuration q in the viewer by placing all the bodies
        """
        if q is not None:
            pin.forwardKinematics(self.model, self.data, q)

        if self.collisionRootFrame.visible:
            self.updatePlacements(pin.GeometryType.COLLISION)

        if self.visualRootFrame.visible:
            self.updatePlacements(pin.GeometryType.VISUAL)

        if self.framesRootFrame.visible:
            self.updateFrames()

    def displayCollisions(self, visibility):
        self.collisionRootFrame.visible = visibility
        self.updatePlacements(pin.GeometryType.COLLISION)

    def displayVisuals(self, visibility):
        self.visualRootFrame.visible = visibility
        self.updatePlacements(pin.GeometryType.VISUAL)

    def displayFrames(self, visibility):
        self.framesRootFrame.visible = visibility
        self.updateFrames()

    def drawFrameVelocities(self, *args, **kwargs):
        raise NotImplementedError("drawFrameVelocities is not yet implemented.")

    def updatePlacements(self, geometry_type):
        if geometry_type == pin.GeometryType.VISUAL:
            geom_model = self.visual_model
            geom_data = self.visual_data
            prefix = self.viewerRootNodeName + "/visual"
        else:
            geom_model = self.collision_model
            geom_data = self.collision_data
            prefix = self.viewerRootNodeName + "/collision"

        pin.updateGeometryPlacements(self.model, self.data, geom_model, geom_data)
        for geom_id, geometry_object in enumerate(geom_model.geometryObjects):
            # Get mesh pose.
            M = geom_data.oMg[geom_id]

            # Update viewer configuration.
            frame_name = prefix + "/" + geometry_object.name
            frame = self.frames[frame_name]
            frame.position = M.translation * geometry_object.meshScale
            frame.wxyz = pin.Quaternion(M.rotation).coeffs()[
                [3, 0, 1, 2]
            ]  # Pinocchio uses xyzw

    def updateFrames(self):
        pin.updateFramePlacements(self.model, self.data)
        for frame_id, frame in enumerate(self.model.frames):
            # Get frame pose.
            M = self.data.oMf[frame_id]

            # Update viewer configuration.
            viser_frame_name = self.framesRootNodeName + "/" + frame.name
            viser_frame = self.frames[viser_frame_name]
            viser_frame.position = M.translation
            viser_frame.wxyz = pin.Quaternion(M.rotation).coeffs()[
                [3, 0, 1, 2]
            ]  # Pinocchio uses xyzw

    def captureImage(self, w=None, h=None, client_id=None, transport_format="jpeg"):
        """
        Capture an image from the Viser viewer and return an RGB array.
        Parameters:
            w: The width of the captured image.
            h: The height of the captured image.
            client_id: The ID of the Viser client handle.
                If None, uses the first available client.
            transport_format: The transport format to use for the captured image.
                Can be "jpeg" (default) or "png".
        """
        clients = self.viewer.get_clients()
        if len(clients) == 0:
            raise RuntimeError("Viser server has no attached clients!")

        if client_id is None:
            cli = next(iter(clients.values()))
        elif client_id not in clients:
            raise RuntimeError(
                f"Viser server does not have a client with ID '{client_id}'"
            )
        else:
            cli = clients[client_id]

        return cli.get_render(height=h, width=w, transport_format=transport_format)

    def setBackgroundColor(self, preset_name: str = "gray", col_top=None, col_bot=None):
        raise NotImplementedError("setBackgroundColor is not yet implemented.")

    def setCameraTarget(self, target: np.ndarray):
        raise NotImplementedError("setCameraTarget is not yet implemented.")

    def setCameraPosition(self, position: np.ndarray):
        raise NotImplementedError("setCameraPosition is not yet implemented.")

    def setCameraZoom(self, zoom: float):
        raise NotImplementedError("setCameraZoom is not yet implemented.")

    def setCameraPose(self, pose: np.ndarray = np.eye(4)):
        raise NotImplementedError("setcameraPose is not yet implemented.")

    def disableCameraControl(self):
        raise NotImplementedError("disableCameraControl is not yet implemented.")

    def enableCameraControl(self):
        raise NotImplementedError("enableCameraControl is not yet implemented.")


def visualizePath(
    viz: ViserVisualizer,
    scene: Scene,
    rrt: RRT,
    path: JointPath | None,
    frame_name: str,
    max_step_size: float,
) -> None:
    """
    Helper function to visualize an RRT path.

    Args
        viz: The viser visualizer instance.
        scene: The scene instance.
        rrt: The RRT planner instance.
        path: The joint path to visualize. If None, does not visualize the path.
        frame_name: The frame name to use for forward kinematics.
        max_step_size: The maximum step size between joint configurations when interpolating paths.
    """
    start_nodes, goal_nodes = rrt.getNodes()

    start_segments = []
    for node in start_nodes[1:]:
        q_start = start_nodes[node.parent_id].config
        q_end = node.config
        frame_path = computeFramePath(scene, q_start, q_end, frame_name, max_step_size)
        for idx in range(len(frame_path) - 1):
            start_segments.append([frame_path[idx][:3, 3], frame_path[idx + 1][:3, 3]])

    goal_segments = []
    for node in goal_nodes[1:]:
        q_start = goal_nodes[node.parent_id].config
        q_end = node.config
        frame_path = computeFramePath(scene, q_start, q_end, frame_name, max_step_size)
        for idx in range(len(frame_path) - 1):
            goal_segments.append([frame_path[idx][:3, 3], frame_path[idx + 1][:3, 3]])

    path_segments = []
    if path is not None:
        for idx in range(len(path.positions) - 1):
            q_start = path.positions[idx]
            q_end = path.positions[idx + 1]
            frame_path = computeFramePath(
                scene, q_start, q_end, frame_name, max_step_size
            )
            for idx in range(len(frame_path) - 1):
                path_segments.append(
                    [frame_path[idx][:3, 3], frame_path[idx + 1][:3, 3]]
                )

    if start_segments:
        viz.viewer.scene.add_line_segments(
            "/rrt/start_tree",
            points=np.array(start_segments),
            colors=(0, 100, 100),
            line_width=1.0,
        )
    if goal_segments:
        viz.viewer.scene.add_line_segments(
            "/rrt/goal_tree",
            points=np.array(goal_segments),
            colors=(100, 0, 100),
            line_width=1.0,
        )

    if path_segments:
        viz.viewer.scene.add_line_segments(
            "/rrt/path",
            points=np.array(path_segments),
            colors=(100, 100, 0),
            line_width=3.0,
        )
