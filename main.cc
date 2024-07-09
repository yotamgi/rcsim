/** Simulator main file. */
#include <irrlicht/irrlicht.h>
#include <sys/time.h>
#include <unistd.h>
#include <iostream>
#include "heli.h"

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
using namespace gui;


/**
 * To receive events like mouse and keyboard input, or GUI events like "the OK
 * button has been clicked", we need an object which is derived from the
 * irr::IEventReceiver object. There is only one method to override:
 * irr::IEventReceiver::OnEvent(). This method will be called by the engine once
 * when an event happens. What we really want to know is whether a key is being
 * held down, and so we will remember the current state of each key.
 */
class EventReceiver : public irr::IEventReceiver
{
public:
    EventReceiver();

    // This is the one method that we have to implement
    virtual bool OnEvent(const irr::SEvent& event);

    // This is used to check whether a key is being held down
    virtual bool IsKeyDown(irr::EKEY_CODE keyCode) const;

private:

    // We use this array to store the current state of each key
    bool KeyIsDown[irr::KEY_KEY_CODES_COUNT];
};

bool EventReceiver::OnEvent(const irr::SEvent& event) {
	// Remember whether each key is down or up
	if (event.EventType == irr::EET_KEY_INPUT_EVENT)
		KeyIsDown[event.KeyInput.Key] = event.KeyInput.PressedDown;

	return false;
}

bool EventReceiver::IsKeyDown(irr::EKEY_CODE keyCode) const {
	return KeyIsDown[keyCode];
}

EventReceiver::EventReceiver()
{
	for (u32 i=0; i<irr::KEY_KEY_CODES_COUNT; ++i)
		KeyIsDown[i] = false;
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
		createDevice( video::EDT_OPENGL, dimension2d<u32>(1024, 768), 32,
			false, false, false, &receiver);

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
	IGUIEnvironment* guienv = device->getGUIEnvironment();

	/*
	We add a hello world label to the window, using the GUI environment.
	The text is placed at the position (10,10) as top left corner and
	(260,22) as lower right corner.
	*/
	guienv->addStaticText(L"Hello World! This is the Irrlicht Software renderer!",
		rect<s32>(10,10,260,22), true);

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

	/*
	To let the mesh look a little bit nicer, we change its material. We
	disable lighting because we do not have a dynamic light in here, and
	the mesh would be totally black otherwise. Then we set the frame loop,
	such that the predefined STAND animation is used. And last, we apply a
	texture to the mesh. Without it the mesh would be drawn using only a
	color.
	*/
	if (node)
	{
		node->setPosition(core::vector3df(0, 0, 0));
		node->setScale(core::vector3df(10, 10, 10));
		node->setMaterialFlag(EMF_LIGHTING, false);
        node->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, false);
        node->setDebugDataVisible(scene::EDS_OFF);
	    node->setMaterialTexture(0, driver->getTexture("media/BasketballStadium/textures/BasketZemin_Color.png"));
	}

    // Init the Heli object.
    HeliParams heli_params;    
    heli_params.shape_path = std::string("media/Bell/source/Copter_2.obj");
    heli_params.texture_path = std::string("media/Bell/textures/1001_albedo.jpg");
    heli_params.shape_scale = irrvec3(0.01, 0.01, 0.01);
    heli_params.init_pos = irrvec3(0, 3, 0);
    heli_params.shape_rotation = irrvec3(0, -90, 0);
    heli_params.init_rotation = irrvec3(0, 0, 0);
    heli_params.swash_sensitivity = 90.;
    heli_params.yaw_sensitivity = 90.;
    heli_params.mass = 1;
    heli_params.max_lift = heli_params.mass * 10 * 3;
    heli_params.drag = irrvec3(0.5, 4, 0.1);

    Heli heli(heli_params, smgr, driver);

    // Add skybox
    smgr->addSkyBoxSceneNode(
		driver->getTexture("media/skybox/irrlicht2_up.jpg"),
		driver->getTexture("media/skybox/irrlicht2_dn.jpg"),
		driver->getTexture("media/skybox/irrlicht2_lf.jpg"),
		driver->getTexture("media/skybox/irrlicht2_rt.jpg"),
		driver->getTexture("media/skybox/irrlicht2_ft.jpg"),
		driver->getTexture("media/skybox/irrlicht2_bk.jpg"));

	irr::scene::ICameraSceneNode* camera_node = device->getSceneManager()->addCameraSceneNode();
    irrvec3 camera_pos(0, 3, -10);
    camera_node->setPosition(camera_pos);

	/*
	Ok, now we have set up the scene, lets draw everything: We run the
	device in a while() loop, until the device does not want to run any
	more. This would be when the user closes the window or presses ALT+F4
	(or whatever keycode closes a window).
	*/
	int lastFPS = -1;
    float lift = 0;
	struct timeval tv;
	gettimeofday(&tv, NULL);
	double then = tv.tv_sec*1000. + tv.tv_usec/1000.;
    double time_delta;

	while(device->run())
	{
        // calculate the delta time, and make sure it does not exceed 100 fps
        double now;
        gettimeofday(&tv, NULL);
        now = tv.tv_sec*1000. +  tv.tv_usec/1000.;
        time_delta = (now - then) / 1000.;
        then = now;

        driver->beginScene(true, true, video::SColor(255,200,200,200));
        smgr->drawAll();
        driver->endScene();

        int fps = driver->getFPS();

        if (lastFPS != fps)
        {
            core::stringw str = L"Irrlicht Engine - Quake 3 Map example [";
            str += driver->getName();
            str += "] FPS:";
            str += fps;

            device->setWindowCaption(str.c_str());
            lastFPS = fps;
        }
        std::cout << "location " << heli.get_position().Y << std::endl;
        
        ///////////////////////////////////////
        // Update the plane according to the keys
        //////////////
        ServoData servo_data;
        if (receiver.IsKeyDown(KEY_UP)) servo_data.pitch = 1.;
        else if (receiver.IsKeyDown(KEY_DOWN)) servo_data.pitch = -1;
        else servo_data.pitch = 0.;

        if (receiver.IsKeyDown(KEY_LEFT))  servo_data.roll = 1.;
        else if (receiver.IsKeyDown(KEY_RIGHT)) servo_data.roll = -1;
        else servo_data.roll = 0;

        if (receiver.IsKeyDown(KEY_KEY_D))  servo_data.yaw = 1.;
        else if (receiver.IsKeyDown(KEY_KEY_A)) servo_data.yaw = -1;
        else servo_data.yaw = 0;

        if (receiver.IsKeyDown(KEY_KEY_W))  lift += time_delta;
        else if (receiver.IsKeyDown(KEY_KEY_S)) lift -= time_delta;
        lift = lift > 1 ? 1 : lift;
        lift = lift < -1 ? -1 : lift;
        servo_data.lift = lift;
        
        heli.update(time_delta, irrvec3(0, 0, -1), servo_data);
        heli.update_ui();
        
        camera_node->setTarget(heli.get_position());

        if (heli.get_position().Y < 0) {
            heli.set_position(irrvec3(heli.get_position().X, 0, heli.get_position().Z));
            heli.set_velocity(irrvec3(heli.get_velocity().X, 0, heli.get_velocity().Z));
        }
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
