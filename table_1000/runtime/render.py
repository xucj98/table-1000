"""Render settings shared by the asset physics tests and the task videos.

RTX Real-Time 2.0 (``RealTimePathTracing``) renders the assets' transmissive
plastics; closed translucent walls need up to 12 transmission bounces. Its
ray-tracing pipelines are built at startup, so RT2 is requested on the command
line (``RT2_ARGS``) only by runs that record video. Lights are a dome plus a sun.

RT2 denoises over time. With one render per captured frame, an object that
moves quickly (a released pen) leaves a trail of its earlier position; rendering
several subframes of the same physical state per capture lets the history settle.
"""

RT2_ARGS = ('--/persistent/rtx/modes/rt2/enabled=true',)
TRANSMISSION_BOUNCES = 12
DOME_INTENSITY, SUN_INTENSITY, SUN_ROTATE_XYZ = 900, 2500, (-40, 20, 0)


def configure():
    """Synchronous capture without motion blur, with enough transmission bounces."""
    import carb
    settings = carb.settings.get_settings()
    for setting in ['/omni/replicator/captureOnPlay', '/omni/replicator/asyncRendering', '/app/asyncRendering',
                    '/rtx/post/motionblur/enabled']:
        settings.set(setting, False)
    for setting in ['/rtx/rtpt/maxBounces', '/rtx/rtpt/maxSpecularAndTransmissionBounces']:
        settings.set(setting, TRANSMISSION_BOUNCES)


def add_lights(stage):
    from pxr import Gf, UsdGeom, UsdLux
    UsdLux.DomeLight.Define(stage, '/World/Light').CreateIntensityAttr(DOME_INTENSITY)
    sun = UsdLux.DistantLight.Define(stage, '/World/Sun')
    sun.CreateIntensityAttr(SUN_INTENSITY)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(*SUN_ROTATE_XYZ))
