/** Simulator main file. */
#include <irrlicht/irrlicht.h>
#include <sys/time.h>
#include <unistd.h>
#include <iostream>
#include "heli.h"
#include "flight_controller.h"
#include "controls.h"
#include "dashboard.h"
#include "input_event_reciever.h"

/*
In the Irrlicht Engine, everything can be found in the namespace 'irr'. So if
you want to use a class of the engine, you have to write irr:: before the name
of the class. For example to use the IrrlichtDevice write: irr::IrrlichtDevice.
To get rid of the irr:: in front of the name of every class, we tell the
compiler that we use that namespace from now on, and we will not have to write
irr:: anymore.
*/
using namespace irr;

/*
There are 5 sub namespaces in the Irrlicht Engine. Take a look at them, you can
read a detailed description of them in the documentation by clicking on the top
menu item 'Namespace List' or by using this link:
http://irrlicht.sourceforge.net/docu/namespaces.html
Like the irr namespace, we do not want these 5 sub namespaces now, to keep this
example simple. Hence, we tell the compiler again that we do not want always to
write their names.
*/
using namespace core;
using namespace scene;
using namespace video;
using namespace io;


void add_banana(irr::scene::ISceneManager *smgr,
                irr::video::IVideoDriver *driver,
                const core::vector3df &position,
                const core::vector3df &rotation)
{
	IMesh* banana_mesh = smgr->getMesh("media/banana/source/banana.obj");
	IMeshSceneNode* banana_node = smgr->addMeshSceneNode(banana_mesh);

    banana_node->setPosition(position);
    banana_node->setRotation(rotation);
    banana_node->setScale(core::vector3df(1, 1, 1)/3);
    banana_node->setMaterialFlag(EMF_LIGHTING, true);
    banana_node->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
    banana_node->setDebugDataVisible(scene::EDS_OFF);
    banana_node->setMaterialTexture(0, driver->getTexture("media/banana/textures/rgb.jpeg"));
    for (unsigned int i=0; i < banana_node->getMaterialCount(); i++) {
        banana_node->getMaterial(i).AmbientColor.set(255, 255, 255, 255);
    }
    banana_node->addShadowVolumeSceneNode();
}

/*
This is the main method. We can now use main() on every platform.
*/
int main()
{
	/*
	The most important function of the engine is the createDevice()
	function. The IrrlichtDevice is created by it, which is the root
	object for doing anything with the engine. createDevice() has 7
	parameters:

	- deviceType: Type of the device. This can currently be the Null-device,
	   one of the two software renderers, D3D8, D3D9, or OpenGL. In this
	   example we use EDT_SOFTWARE, but to try out, you might want to
	   change it to EDT_BURNINGSVIDEO, EDT_NULL, EDT_DIRECT3D8,
	   EDT_DIRECT3D9, or EDT_OPENGL.

	- windowSize: Size of the Window or screen in FullScreenMode to be
	   created. In this example we use 640x480.

	- bits: Amount of color bits per pixel. This should be 16 or 32. The
	   parameter is often ignored when running in windowed mode.

	- fullscreen: Specifies if we want the device to run in fullscreen mode
	   or not.

	- stencilbuffer: Specifies if we want to use the stencil buffer (for
	   drawing shadows).

	- vsync: Specifies if we want to have vsync enabled, this is only useful
	   in fullscreen mode.

	- eventReceiver: An object to receive events. We do not want to use this
	   parameter here, and set it to 0.

	Always check the return value to cope with unsupported drivers,
	dimensions, etc.
	*/
	EventReceiver receiver;
	IrrlichtDevice *device =
		createDevice( video::EDT_OPENGL, dimension2d<u32>(1600, 1200), 32,
			false, true, false, &receiver);

	if (!device)
		return 1;

	/*
	Set the caption of the window to some nice text. Note that there is an
	'L' in front of the string. The Irrlicht Engine uses wide character
	strings when displaying text.
	*/
	device->setWindowCaption(L"My RC Helicopter Simulator");

	/*
	Get a pointer to the VideoDriver, the SceneManager and the graphical
	user interface environment, so that we do not always have to write
	device->getVideoDriver(), device->getSceneManager(), or
	device->getGUIEnvironment().
	*/
	IVideoDriver* driver = device->getVideoDriver();
	ISceneManager* smgr = device->getSceneManager();

	/*
	We add a hello world label to the window, using the GUI environment.
	The text is placed at the position (10,10) as top left corner and
	(260,22) as lower right corner.
	*/

    // Light configuration.
	smgr->setAmbientLight(video::SColorf(0.5, 0.5, 0.5));
    smgr->addLightSceneNode(0, core::vector3df(-200, 100, -200),
		video::SColorf(0.3, 0.3, 0.3),2000);
    smgr->setShadowColor(video::SColor(80,0,0,0));


	/*
	To show something interesting, we load a Quake 2 model and display it.
	We only have to get the Mesh from the Scene Manager with getMesh() and add
	a SceneNode to display the mesh with addAnimatedMeshSceneNode(). We
	check the return value of getMesh() to become aware of loading problems
	and other errors.

	Instead of writing the filename sydney.md2, it would also be possible
	to load a Maya object file (.obj), a complete Quake3 map (.bsp) or any
	other supported file format. By the way, that cool Quake 2 model
	called sydney was modelled by Brian Collins.
	*/
	IMesh* mesh = smgr->getMesh("media/BasketballStadium/source/63BasketBallZemin.obj");
	if (!mesh)
	{
		device->drop();
		return 1;
	}
	IMeshSceneNode* node = smgr->addMeshSceneNode(mesh);

    node->setPosition(core::vector3df(0, 0, 0));
    node->setScale(core::vector3df(4, 4, 4));
    node->setMaterialFlag(EMF_LIGHTING, true);
    node->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
    node->setDebugDataVisible(scene::EDS_OFF);
    node->setMaterialTexture(0, driver->getTexture("media/BasketballStadium/textures/BasketZemin_Color.png"));
    for (unsigned int i=0; i < node->getMaterialCount(); i++) {
        node->getMaterial(i).AmbientColor.set(255, 255, 255, 255);
    }

    add_banana(smgr, driver, core::vector3df(-0.5, 0.05, 0.2), core::vector3df(90, 73, 0));
    add_banana(smgr, driver, core::vector3df(0.9, 0.05, 0.3), core::vector3df(90, 40, 0));
    add_banana(smgr, driver, core::vector3df(0.4, 0.05, 0.0), core::vector3df(90, 0, 0));

    // The helicopter setup.
    BellHeli heli(smgr, driver);
    GyroFlightController flight_controller(&heli);
    Controls controls(
            &flight_controller,
            // Throttle curves:
            {
                // Normal mode:
                ControllerCurve({
                    ControllerCurve::Point(-1, -1),
                    ControllerCurve::Point(-0.5, 0.2),
                    ControllerCurve::Point(1, 0.2),
                }),
                // Idle-up mode:
                ControllerCurve({
                    ControllerCurve::Point(-1, 0.5),
                    ControllerCurve::Point( 1, 0.5),
                }),
            },
            // Blades pitch curves: 
            {
                // Normal mode:
                ControllerCurve({
                    ControllerCurve::Point(-1, -0.1),
                    ControllerCurve::Point(-0.5, -0.0),
                    ControllerCurve::Point( 1, 1.),
                }),
                // Idle-up mode:
                ControllerCurve({
                    ControllerCurve::Point(-1, -1),
                    ControllerCurve::Point( 1, 1),
                }),
            }
    );
    Dashboard dashboard(driver, 
		controls.get_throttle_curves(), 
		controls.get_lift_curves(), 
		heli.get_params().main_rotor_max_vel
	);

    // Add skybox
    smgr->addSkyBoxSceneNode(
		driver->getTexture("media/skybox/irrlicht2_up.jpg"),
		driver->getTexture("media/skybox/irrlicht2_dn.jpg"),
		driver->getTexture("media/skybox/irrlicht2_lf.jpg"),
		driver->getTexture("media/skybox/irrlicht2_rt.jpg"),
		driver->getTexture("media/skybox/irrlicht2_ft.jpg"),
		driver->getTexture("media/skybox/irrlicht2_bk.jpg"));

	irr::scene::ICameraSceneNode* camera_node = device->getSceneManager()->addCameraSceneNode();
    irrvec3 camera_pos(0, 1.5, -3);
    camera_node->setPosition(camera_pos);

	/*
	Ok, now we have set up the scene, lets draw everything: We run the
	device in a while() loop, until the device does not want to run any
	more. This would be when the user closes the window or presses ALT+F4
	(or whatever keycode closes a window).
	*/
	int lastFPS = -1;
	struct timeval tv;
	gettimeofday(&tv, NULL);
	double then = tv.tv_sec*1000. + tv.tv_usec/1000.;
    double time_delta;


    irr::core::array<SJoystickInfo> joystickInfo;
    device->activateJoysticks(joystickInfo);

	while(device->run())
	{
        // calculate the delta time, and make sure it does not exceed 100 fps
        double now;
        gettimeofday(&tv, NULL);
        now = tv.tv_sec*1000. +  tv.tv_usec/1000.;
        time_delta = (now - then) / 1000.;
        then = now;
        time_delta = time_delta > 0.03 ? 0.03 : time_delta;


        // Show FPS.
        int fps = driver->getFPS();
        if (lastFPS != fps)
        {
            core::stringw str = L"RC Heli Simulator";
            str += driver->getName();
            str += "] FPS:";
            str += fps;

            device->setWindowCaption(str.c_str());
            lastFPS = fps;
        }

        // Update the helicopter.
        UserInput user_input = receiver.update_input(time_delta);
        heli.update(
                time_delta,
                irrvec3(0, 0, 0),
                controls.get_servo_data(user_input.controls_input, time_delta)
        );

        // Apply external force on the helicopter touch points.
        heli.reset_force();
        std::vector<BaseHeli::TouchPoint> touchpoints = heli.get_touchpoints_in_world();
        for (unsigned int i=0; i<touchpoints.size(); i++) {
            BaseHeli::TouchPoint tp = touchpoints[i];
            if (tp.pos_in_world.Y < 0) {
                irrvec3 tp_force = irrvec3(0, -500*tp.pos_in_world.Y, 0);
                tp_force += - tp.vel_in_world * irrvec3(15, 10, 15) * (-tp.pos_in_world.Y / 0.02);
                heli.add_force(i, tp_force);
            }
        }

        // Draw.
        camera_node->setTarget(heli.get_position());
        driver->beginScene(true, true, video::SColor(255,200,200,200));
        smgr->drawAll();
        dashboard.update_ui(controls.get_telemetry(), heli.get_telemetry());
        driver->endScene();
	}

	/*
	After we are done with the render loop, we have to delete the Irrlicht
	Device created before with createDevice(). In the Irrlicht Engine, you
	have to delete all objects you created with a method or function which
	starts with 'create'. The object is simply deleted by calling ->drop().
	See the documentation at irr::IReferenceCounted::drop() for more
	information.
	*/
	device->drop();

	return 0;
}

/*
That's it. Compile and run.
**/
