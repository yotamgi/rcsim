/** Simulator main file. */
#include <irrlicht/irrlicht.h>
#include <sys/time.h>
#include <unistd.h>
#include <iostream>
#include "heli.h"
#include "controller.h"

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

    ServoData get_servo_data(float time_delta);

private:

    void update_value(double &value, irr::EKEY_CODE key_up, irr::EKEY_CODE key_down, float change_amount);

    // We use this array to store the current state of each key
    bool KeyIsDown[irr::KEY_KEY_CODES_COUNT];

    SEvent::SJoystickEvent JoystickState;
    bool m_joystick_active;
    ServoData m_servo_data;
};

EventReceiver::EventReceiver()
{
	for (u32 i=0; i<irr::KEY_KEY_CODES_COUNT; ++i)
		KeyIsDown[i] = false;
    m_joystick_active = false;
    m_servo_data.pitch = 0;
    m_servo_data.lift = 0;
    m_servo_data.roll = 0;
    m_servo_data.yaw = 0;
}

bool EventReceiver::OnEvent(const irr::SEvent& event) {
	// Remember whether each key is down or up
	if (event.EventType == irr::EET_KEY_INPUT_EVENT)
		KeyIsDown[event.KeyInput.Key] = event.KeyInput.PressedDown;

    // The state of each connected joystick is sent to us
    // once every run() of the Irrlicht device.  Store the
    // state of the first joystick, ignoring other joysticks.
    // This is currently only supported on Windows and Linux.
    if (event.EventType == irr::EET_JOYSTICK_INPUT_EVENT
        && event.JoystickEvent.Joystick == 0)
    {
        JoystickState = event.JoystickEvent;
        m_joystick_active = true;
    }

	return false;
}

void EventReceiver::update_value(double &value, irr::EKEY_CODE key_up, irr::EKEY_CODE key_down, float change_amount) {
    if (IsKeyDown(key_up)) value += change_amount;
    else if (IsKeyDown(key_down)) value -= change_amount;
    else {
        if (std::abs(value) < 0.1) value = 0;
        else {
            value += change_amount * ((float)(value < 0)*2 - 1);
        }
    }

    value = value > 1 ? 1 : value;
    value = value < -1 ? -1 : value;
}

ServoData EventReceiver::get_servo_data(float time_delta) {
    if (m_joystick_active) {
        m_servo_data.pitch = -(float)JoystickState.Axis[1] / 32768;
        m_servo_data.roll = -(float)JoystickState.Axis[0] / 32768;
        m_servo_data.yaw = (float)JoystickState.Axis[4] / 32768;
        m_servo_data.lift = -(float)JoystickState.Axis[2] / 32768;
        m_servo_data.throttle = 1;
        return m_servo_data;
    }

    float change_amount = time_delta * 2;
    update_value(m_servo_data.pitch, KEY_UP, KEY_DOWN, change_amount);
    update_value(m_servo_data.roll, KEY_LEFT, KEY_RIGHT, change_amount);
    update_value(m_servo_data.yaw, KEY_KEY_D, KEY_KEY_A, change_amount);

    if (IsKeyDown(KEY_KEY_W))  m_servo_data.lift += time_delta;
    else if (IsKeyDown(KEY_KEY_S)) m_servo_data.lift -= time_delta;

    m_servo_data.lift = m_servo_data.lift > 1 ? 1 : m_servo_data.lift;
    m_servo_data.lift = m_servo_data.lift < -1 ? -1 : m_servo_data.lift;
    m_servo_data.throttle = 0.75;
    return m_servo_data;
}

bool EventReceiver::IsKeyDown(irr::EKEY_CODE keyCode) const {
	return KeyIsDown[keyCode];
}

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
	IGUIEnvironment* guienv = device->getGUIEnvironment();

	/*
	We add a hello world label to the window, using the GUI environment.
	The text is placed at the position (10,10) as top left corner and
	(260,22) as lower right corner.
	*/
	guienv->addStaticText(L"Hello World! This is the Irrlicht Software renderer!",
		rect<s32>(10,10,260,22), true);

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
    node->setScale(core::vector3df(5, 5, 5));
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


    // Init the Heli object.
    BellHeli heli(smgr, driver);

    // Init the controller.
    TailGyroController controller(&heli);

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

        ///////////////////////////////////////
        // Update the plane according to the keys
        //////////////
        ServoData servo_data = receiver.get_servo_data(time_delta);
        servo_data = controller.updateServoData(servo_data, time_delta);
        
        heli.update(time_delta, irrvec3(0, 0, -1), servo_data);
        
        camera_node->setTarget(heli.get_position());

        if (heli.get_position().Y < 0.125) {
            heli.set_position(irrvec3(heli.get_position().X, 0.125, heli.get_position().Z));
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
