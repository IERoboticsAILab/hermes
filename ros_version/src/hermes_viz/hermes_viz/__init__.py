"""H.E.R.M.E.S 3D visualization package.

Vuer reference — API surface verified against vuer 0.1.6 (installed via pip install --user vuer,
Python 3.12, miniforge environment on macOS Darwin 24.6.0).

Install method that worked:
    pip install --user vuer          # installs to ~/.local/lib/python3.12/site-packages
    # miniforge Python must include the --user path in sys.path (it does by default)
    # Or: pip install --target /path/to/miniforge3/lib/python3.12/site-packages vuer

-------------------------------------------------------------------------------
Top-level imports
-------------------------------------------------------------------------------

    from vuer import Vuer           # server / app class
    from vuer import VuerSession    # per-client session (passed to spawn handlers)
    from vuer import VuerClient     # client-only mode (no server)
    from vuer import Workspace      # workspace management
    import vuer.schemas             # all scene-graph node types

-------------------------------------------------------------------------------
Server lifecycle
-------------------------------------------------------------------------------

    app = Vuer(port=8012)           # create server (default port 8012)

    @app.spawn(start=True)          # decorator — registers async handler, starts server
    async def main(sess: VuerSession):
        ...

    app.run()                       # alternative: run without decorator

    # Vuer prints these URLs on start:
    #   Local:   https://vuer.ai?ws=ws://localhost:<port>
    #   Network: https://vuer.ai?ws=ws://<lan-ip>:<port>

    # Other Vuer methods/properties used by hermes_viz:
    #   app.spawn       — decorator factory
    #   app.add_handler — register event handler
    #   app.add_route   — add HTTP route
    #   app.send        — send event to all clients
    #   app.uplink      — send to server queue
    #   app.downlink    — receive from server queue
    #   app.relay       — relay events between queues
    #   app.rpc         — remote procedure call helper
    #   app.rpc_stream  — streaming RPC helper

-------------------------------------------------------------------------------
Session API  (VuerSession instance passed to @app.spawn handler)
-------------------------------------------------------------------------------

    # Scene-mutation operators — all use Python matmul (@) as "send to":
    sess.set @ <node>       # replace root scene entirely
    sess.upsert @ <node>    # add-or-update node by key (partial update)
    sess.update @ <node>    # update existing node props (no add)
    sess.add @ <node>       # append node as child

    # set/upsert/update/add are properties returning a MatMul descriptor;
    # the @ operator triggers an async send to the connected browser client.

    # Other session methods:
    sess.remove(key)        # remove node by key
    sess.clear()            # clear scene
    sess.send(event)        # send raw event
    sess.rpc(event)         # blocking RPC call
    sess.stream(event)      # streaming RPC
    sess.grab_render()      # capture browser render
    sess.till(event_type)   # wait for event
    sess.forever()          # keep session alive indefinitely
    sess.spawn_task(coro)   # schedule background coroutine

-------------------------------------------------------------------------------
Schema classes confirmed present in vuer.schemas (vuer 0.1.6)
-------------------------------------------------------------------------------

Primitives / geometry:
    Box, Sphere, Plane, Capsule, Cylinder, Cone, Circle, Ring, Torus, TorusKnot
    Dodecahedron, Icosahedron, Octahedron, Tetrahedron, Polyhedron
    Tube, Extrude, Lathe, Shape

3D mesh / point-cloud:
    TriMesh         # indexed triangle mesh (vertices + faces as numpy arrays)
    Pcd             # point-cloud (PCD format)
    PointCloud      # point-cloud (xyz array)
    DepthPointCloud # RGBD depth point cloud
    Mesh            # NOT present — use TriMesh instead

File loaders:
    Glb             # GLB / GLTF binary  (NOTE: 'Gltf' is NOT exported; use Glb)
    Obj             # Wavefront OBJ
    Stl             # STL
    Dae             # Collada DAE
    Fbx             # FBX
    Urdf            # URDF robot model
    Ply             # PLY point cloud / mesh
    Splat           # 3D Gaussian splat

Lights:
    AmbientLight, DirectionalLight, PointLight, SpotLight
    HemisphereLight, RectAreaLight
    AmbientLightStage, HemisphereLightStage

Cameras:
    PerspectiveCamera, OrthographicCamera
    SimplePerspectiveCamera, SimpleOrthographicCamera
    CameraView, SceneCamera, SceneCameraControl, MjCameraView

Scene containers:
    Scene           # root scene node (wraps all 3D content)
    DefaultScene    # scene with sensible defaults (lights, camera, grid)
    Group           # grouping node (transform parent)
    Movable         # draggable group node
    SceneElement    # base class

HTML / 2D overlay:
    Html            # embed HTML in 3D scene
    Iframe
    Markdown
    Image, Img
    Button, InputBox, Slider
    Header, Header1, Header2, Header3
    Paragraph, Bold, Italic
    BlockElement, AutoScroll, Page

Lines / arrows:
    Line
    Arrow
    CatmullRomLine, CubicBezierLine, QuadraticBezierLine

Materials:
    MeshStandardMaterial, MeshPhysicalMaterial, MeshBasicMaterial
    MeshLambertMaterial, MeshPhongMaterial, MeshToonMaterial
    MeshNormalMaterial, MeshDepthMaterial, MeshMatcapMaterial
    PointsMaterial, LineBasicMaterial, LineDashedMaterial
    ShaderMaterial, RawShaderMaterial, ShadowMaterial, SpriteMaterial
    VideoMaterial, WebRTCVideoMaterial

Controls / interaction:
    OrbitControls, PointerControls
    Clickable, Pivot, Edges, Wireframe
    Gamepad, KeyboardMonitor, MotionControllers, Hands, HandActuator

Text:
    Text, Text3D, Billboard

Misc:
    Grid, CoordsMarker, BBox, BoundingBox, Fog, Stage
    Trail, Path, Frustum, OrthographicFrustum, PerspectiveFrustum
    ThreeAnimate, PlaybackAnimate, AnimationClip
    ViewportHud, HUDPlane

-------------------------------------------------------------------------------
Missing names (were in the plan spec but NOT in vuer 0.1.6):
-------------------------------------------------------------------------------

    Gltf  — use Glb instead (same format, different name)
    Mesh  — use TriMesh for indexed meshes

-------------------------------------------------------------------------------
Typical usage pattern for hermes_viz
-------------------------------------------------------------------------------

    from vuer import Vuer, VuerSession
    from vuer.schemas import (
        Scene, DefaultScene, Group, Box, Sphere, TriMesh, PointCloud, Pcd,
        Line, Arrow, CoordsMarker, AmbientLight, DirectionalLight,
        Glb, Urdf, Html, Text, Grid,
    )
    import asyncio

    app = Vuer(port=8012)

    @app.spawn(start=True)
    async def main(sess: VuerSession):
        sess.set @ DefaultScene()          # initialise scene
        sess.upsert @ Group(               # add / update a robot model
            Glb(src="/path/to/model.glb"),
            key="robot_0",
        )
        while True:
            sess.update @ Group(           # update pose each tick
                position=[x, y, z],
                rotation=[rx, ry, rz],
                key="robot_0",
            )
            await asyncio.sleep(0.05)

"""
