//==============================================================================
/*
    \author    Your Name
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "chai3d.h"
#include "minion.h"
#include "ForceFieldSphere.h"
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------
#ifndef MACOSX
#include "GL/glut.h"
#else
#include "GLUT/glut.h"
#endif
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------


cVector3d g = cVector3d(0, 0, -9.81);
double airC = 0.5;// N/s
double springLength = 0.06; //m
cVector3d force(0, 0, 0);

struct point
{
	double mass = 0.2;
	double k = 400; //   N/m
	double springC = 10; // Ns/m
	cVector3d anchor = cVector3d(0, -0.0375, 0.05);
	cVector3d pposition = cVector3d(0, 0, 0); //m
	cVector3d pvelocity = cVector3d(0, 0, 0); //n/s
	cShapeLine* spring;

	point(double m, double thek, double c, cVector3d a, cShapeLine * s)
	{
		mass = m;
		k = thek;
		springC = c;
		anchor = a;
		spring = s;
	}

	void computeSpringForces(cVector3d position, ForceFieldSphere* particle, double delta_t)
	{
		cVector3d normal = pposition - anchor;
		normal.normalize();
		cVector3d forceParticle = -k * (pposition - anchor - normal * springLength) - springC * normal * pvelocity.dot(normal) - airC * pvelocity;

		cVector3d theForce = particle->calculateForce(position);

		force = force + theForce;
		force -= forceParticle;

		forceParticle = forceParticle + -theForce;


		// Explicit euler integration
		cVector3d acc = forceParticle / mass + g;
		cVector3d vel = pvelocity + delta_t * acc;
		cVector3d pos = pposition + delta_t * pvelocity;

		pposition = pos;
		pvelocity = vel;
	}
};
// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled 
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;


//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelHapticRate;

// a label to display money count;
cLabel* moneyLabel;

// a small sphere (cursor) representing the haptic device 
//cShapeSphere* cursor;

cToolCursor* tool;

// flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// flag to indicate if the haptic simulation has terminated
bool simulationFinished = false;

// frequency counter to measure the simulation haptic rate
cFrequencyCounter frequencyCounter;

// information about computer screen and GLUT display window
int screenW;
int screenH;
int windowW;
int windowH;
int windowPosX;
int windowPosY;

// testing floor
//cShapeBox* testFloor;
cMultiMesh* cake;
cMesh* table;
double toolRadius = 0.0025;

cShapeCylinder* fakeSpawns[8];
minion* fakeAnts[16];
double previousT = 0;
double previousT2 = 0;
double spawnRadius = 0.05;

cShapeCylinder* enlargeButton;


// Justin's part
//------------------------------------------------------------------------------
cMultiMesh *monkey;
cMultiMesh *monkey2;
int hitpoints = 3;
bool touched = false;
cGenericObject* object;
double topOffset = 0.004;
double bottomOffset = 0.0025;

struct unit
{
	double reward;
	double cost;
	double speed;
	int hitpoints;
	cMultiMesh * mesh;
	cMultiMesh * bound;
	string type;
};

vector<unit*> minions;
vector<unit*> objects;
//unit* minions[16];
//double toolRadius = 0.002;
int money = 0;
unit* held;
int holding = 0;

int pushed = 0;
double mass = 0.5;
int counter = 0;

cVector3d torque(0, 0, 0);
double gripperForce = 0.0;

cMultiMesh * cakeBox;
int loop = 0;

bool hit = false;


point* objectWeight;
double spring = 400.0;
double springDamp = 1;
//cShapeLine* spring1;
ForceFieldSphere* objectWeightHolder;


ForceFieldSphere* particle1;
cShapeLine* spring1;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void resizeWindow(int w, int h);

// callback when a key is pressed
void keySelect(unsigned char key, int x, int y);

// callback to render graphic scene
void updateGraphics(void);

// callback of GLUT timer
void graphicsTimer(int data);

// function that closes the application
void close(void);

// main haptics simulation loop
void updateHaptics(void);

void changeCollisionDetector(string flag)
{
	if (flag == "off")
		for (int i = 0; i < minions.size(); i++)
		{
			minions[i]->mesh->deleteCollisionDetector();
		}
	else
		for (int i = 0; i < minions.size(); i++)
			minions[i]->mesh->createAABBCollisionDetector(toolRadius);

}


//==============================================================================
/*
    TEMPLATE:    application.cpp

    Description of your application.
*/
//==============================================================================

int main(int argc, char* argv[])
{
    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[x] - Exit application" << endl;
    cout << endl << endl;


    //--------------------------------------------------------------------------
    // OPENGL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLUT
    glutInit(&argc, argv);

    // retrieve  resolution of computer display and position window accordingly
    screenW = glutGet(GLUT_SCREEN_WIDTH);
    screenH = glutGet(GLUT_SCREEN_HEIGHT);
    windowW = (int)(0.8 * screenH);
    windowH = (int)(0.5 * screenH);
    windowPosY = (screenH - windowH) / 2;
    windowPosX = windowPosY; 

    // initialize the OpenGL GLUT window
    glutInitWindowPosition(windowPosX, windowPosY);
    glutInitWindowSize(windowW, windowH);

    if (stereoMode == C_STEREO_ACTIVE)
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE | GLUT_STEREO);
    else
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);

    // create display context and initialize GLEW library
    glutCreateWindow(argv[0]);

#ifdef GLEW_VERSION
    // initialize GLEW
    glewInit();
#endif

    // setup GLUT options
    glutDisplayFunc(updateGraphics);
    glutKeyboardFunc(keySelect);
    glutReshapeFunc(resizeWindow);
    glutSetWindowTitle("CHAI3D");

    // set fullscreen mode
    if (fullscreen)
    {
        glutFullScreen();
    }


    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setWhite();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and orient the camera
    camera->set( cVector3d (0.1, 0.0, 0.05),    // camera position (eye)
                 cVector3d (0.0, 0.0, 0.0),    // look at position (target)
                 cVector3d (0.0, 0.0, 1.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.01);
    camera->setStereoFocalLength(0.5);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a directional light source
    light = new cDirectionalLight(world);

    // insert light source inside world
    world->addChild(light);

    // enable light source
    light->setEnabled(true);

    // define direction of light beam
    light->setDir(-1.0,-1.0,-1.0); 

	

    //--------------------------------------------------------------------------
    // HAPTIC DEVICE / TOOLS
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get a handle to the first haptic device
    handler->getDevice(hapticDevice, 0);

    // open a connection to haptic device
    hapticDevice->open();

    // calibrate device (if necessary)
    hapticDevice->calibrate();

    // retrieve information about the current haptic device
    cHapticDeviceInfo info = hapticDevice->getSpecifications();

	// create a 3D tool and add it to the world
	tool = new cToolCursor(world);
	world->addChild(tool);
	tool->enableDynamicObjects(true);

	// position tool in respect to camera
	//tool->setLocalPos(0.0, 0.0, 0.0);

	// connect the haptic device to the tool
	tool->setHapticDevice(hapticDevice);
	
	// define a radius for the tool
	tool->setRadius(toolRadius);

	// map the physical workspace of the haptic device to a larger virtual workspace.
	//tool->setWorkspaceRadius(0.2);

	// start the haptic tool
	tool->start();

    // if the device has a gripper, enable the gripper to simulate a user switch
    hapticDevice->setEnableGripperUserSwitch(true);


	//--------------------------------------------------------------------------
	// CREATE OBJECTS
	//--------------------------------------------------------------------------

	// read the scale factor between the physical workspace of the haptic
	// device and the virtual workspace defined for the tool
	double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

	// properties
	double maxStiffness = info.m_maxLinearStiffness / workspaceScaleFactor;


	table = new cMesh();
	cCreatePlane(table, 0.3, 0.3);
	table->createAABBCollisionDetector(toolRadius);
	world->addChild(table);
	table->m_texture = cTexture2d::create();
	table->m_texture->loadFromFile("woodTexture.jpg");
	table->setUseTexture(true);
	table->m_material->setWhite();

	cNormalMapPtr normalTable = cNormalMap::create();
	normalTable->createMap(table->m_texture);
	table->m_normalMap = normalTable;
	table->m_material->setStiffness(0.6 * maxStiffness);
	table->m_material->setStaticFriction(0.62);
	table->m_material->setDynamicFriction(0.48);

	cake = new cMultiMesh;
	cake->loadFromFile("UglyCake.obj");
	cake->scale(0.015);
	world->addChild(cake);

	cakeBox = new cMultiMesh;
	cakeBox->loadFromFile("Ugly_Cake_Bounding_Box.obj");
	cakeBox->scale(0.015);
	world->addChild(cakeBox); 
	cakeBox->setTransparencyLevel(0);
	
	cakeBox->createAABBCollisionDetector(toolRadius);
	cakeBox->setStiffness(0.3 * maxStiffness);

	enlargeButton = new cShapeCylinder(0.003, 0.003, 0.003);
	enlargeButton->m_material->setRed();
	enlargeButton->setLocalPos(-0.05, 0.05, 0);
	world->addChild(enlargeButton);
	
	objectWeightHolder = new ForceFieldSphere(0.01);
	objectWeightHolder->setTransparencyLevel(0);
	world->addChild(objectWeightHolder);

	/*spring1 = new cShapeLine();
	world->addChild(spring1);*/

	particle1 = new ForceFieldSphere(0.01);
	particle1->m_material->setGrayGainsboro();
	world->addChild(particle1);

	spring1 = new cShapeLine();
	world->addChild(spring1);

	// spawn point
	for (int i = 0; i < 8; i++){
		cShapeCylinder* spawn = new cShapeCylinder(0.003, 0.003, 0.0001);
		double degToRad = (double)(i * 45) / 180 * C_PI;
		spawn->m_material->setBlack();
		spawn->setLocalPos(cos(degToRad)*spawnRadius, sin(degToRad)*spawnRadius, 0);
		fakeSpawns[i] = spawn;
		world->addChild(spawn);
	}

	// creating ants
	//for (int i = 0; i < 16; i++){
	//	minion* ant = new minion(0.002);
	//	ant->health = 10;
	//	ant->speed = 0.0003;
	//	double degToRad = (double)(i * 45) / 180 * C_PI;
	//	ant->m_material->setRedCrimson();
	//	ant->setLocalPos(cos(degToRad)*spawnRadius, sin(degToRad)*spawnRadius, 0.001);
	//	fakeAnts[i] = ant;
	//	world->addChild(ant);
	//}

	for (int i = 0; i < 16; i++){
		monkey = new cMultiMesh();
		monkey->loadFromFile("Ant_Bounding_Box.obj");
		unit *enemy = new unit;
		enemy->mesh = monkey;
		enemy->hitpoints = 5;
		enemy->speed = 0.0003;
		enemy->mesh->scale(0.005);
		enemy->mesh->rotateAboutLocalAxisDeg(0, 0, 1, i * 45);
		enemy->mesh->rotateAboutLocalAxisDeg(0, 0, 1, 90);
		enemy->type = "normal";
		
		monkey2 = new cMultiMesh();
		monkey2->loadFromFile("90809.obj");

		enemy->bound = monkey2;
		//enemy->bound->loadFromFile("Ant_Skin2.obj");
		enemy->mesh->createAABBCollisionDetector(toolRadius);
		enemy->mesh->setStiffness(500, true);
		enemy->mesh->computeBoundaryBox(true);
		enemy->bound->scale(0.005);
		enemy->bound->rotateAboutLocalAxisDeg(0, 0, 1, i * 45);
		enemy->bound->rotateAboutLocalAxisDeg(0, 0, 1, 90);

		enemy->mesh->setTransparencyLevel(0);

		//enemy->mesh->createAABBCollisionDetector(toolRadius);
		//enemy->mesh->setStiffness(500, true);
		//enemy->mesh->computeBoundaryBox(true);
		double degToRad = (double)(i * 45) / 180 * C_PI;

		enemy->mesh->setLocalPos(cos(degToRad)*spawnRadius, sin(degToRad)*spawnRadius, 0.001);
		enemy->bound->setLocalPos(cos(degToRad)*spawnRadius, sin(degToRad)*spawnRadius, 0.001);

		minions.push_back(enemy);
		world->addChild(minions[i]->bound);
		world->addChild(minions[i]->mesh);
		

	}



    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    cFont *font = NEW_CFONTCALIBRI20();
    
    // create a label to display the haptic rate of the simulation
    labelHapticRate = new cLabel(font);
    labelHapticRate->m_fontColor.setRed();
    camera->m_frontLayer->addChild(labelHapticRate);

	//--------------------------------------------------------------------------
	// UI
	//--------------------------------------------------------------------------
	moneyLabel = new cLabel(font);
	moneyLabel->m_fontColor.setRed();
	camera->m_frontLayer->addChild(moneyLabel);



    //--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    cThread* hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);

    // start the main graphics rendering loop
    glutTimerFunc(50, graphicsTimer, 0);
    glutMainLoop();

    // exit
    return (0);
}

//------------------------------------------------------------------------------

void resizeWindow(int w, int h)
{
    windowW = w;
    windowH = h;
}

//------------------------------------------------------------------------------

void keySelect(unsigned char key, int x, int y)
{
    // option ESC: exit
    if ((key == 27) || (key == 'x'))
    {
        close();
        exit(0);
    }

    // option f: toggle fullscreen
    if (key == 'f')
    {
        if (fullscreen)
        {
            windowPosX = glutGet(GLUT_INIT_WINDOW_X);
            windowPosY = glutGet(GLUT_INIT_WINDOW_Y);
            windowW = glutGet(GLUT_INIT_WINDOW_WIDTH);
            windowH = glutGet(GLUT_INIT_WINDOW_HEIGHT);
            glutPositionWindow(windowPosX, windowPosY);
            glutReshapeWindow(windowW, windowH);
            fullscreen = false;
        }
        else
        {
            glutFullScreen();
            fullscreen = true;
        }
    }

    // option m: toggle vertical mirroring
    if (key == 'm')
    {
        mirroredDisplay = !mirroredDisplay;
        camera->setMirrorVertical(mirroredDisplay);
    }

	if (key == 'r'){
		for (int i = 0; i < minions.size(); i++){
			double degToRad = (double)(i * 45) / 180 * C_PI;
			minions[i]->mesh->setLocalPos(cos(degToRad)*spawnRadius, sin(degToRad)*spawnRadius, 0.001);
		}
	}

	if (key == 'q'){
		toolRadius += 0.0005;
		tool->setRadius(toolRadius);
		table->deleteCollisionDetector();
		table->createAABBCollisionDetector(toolRadius);
		//money -= 5;
	}

	if (key == 'a'){
		toolRadius -= 0.0005;
		tool->setRadius(toolRadius);
		table->deleteCollisionDetector();
		table->createAABBCollisionDetector(toolRadius);
	}

}

//------------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    hapticDevice->close();
}

//------------------------------------------------------------------------------

void graphicsTimer(int data)
{
    if (simulationRunning)
    {
        glutPostRedisplay();
    }

    glutTimerFunc(50, graphicsTimer, 0);
}

//------------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // display haptic rate data
    labelHapticRate->setText(cStr(frequencyCounter.getFrequency(), 0) + " Hz");

    // update position of label
    labelHapticRate->setLocalPos((int)(0.5 * (windowW - labelHapticRate->getWidth())), 15);


	/////////////////////////////////////////////////////////////////////
	// UPDATE UI
	/////////////////////////////////////////////////////////////////////

	// display money amount
	moneyLabel->setText("Money: " + cStr((double)money, 0));
	
	// update position of label
	moneyLabel->setLocalPos((int)(0.3 * (windowW - moneyLabel->getWidth())), 15);

    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, mirroredDisplay);

    // render world
    camera->renderView(windowW, windowH);

    // swap buffers
    glutSwapBuffers();

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error:  %s\n" << gluErrorString(err);
}


// actually it moves the ants
void calcAntPos(unit* object){
	
	double stepsize = object->speed;
	cVector3d antPos = object->mesh->getLocalPos();

	double distance = sqrt(pow(antPos.x(), 2) + pow(antPos.y(), 2));
	double angle = atan(antPos.x() / antPos.y());
	double xsign = antPos.x() / abs(antPos.x());
	double ysign = antPos.y() / abs(antPos.y());


	if (antPos.y() < 0){
		if (antPos.x() >= 0){
			object->mesh->setLocalPos((distance - stepsize)*sin(angle)* ysign, (distance - stepsize)*cos(angle) * ysign, 0.001);
			object->bound->setLocalPos((distance - stepsize)*sin(angle)* ysign, (distance - stepsize)*cos(angle) * ysign, 0.001);

		}
		else{
			object->mesh->setLocalPos((distance - stepsize)*sin(angle)*xsign, (distance - stepsize)*cos(angle) * ysign, 0.001);
			object->bound->setLocalPos((distance - stepsize)*sin(angle)*xsign, (distance - stepsize)*cos(angle) * ysign, 0.001);

		}
	}
	else{
		object->mesh->setLocalPos((distance - stepsize)*sin(angle), (distance - stepsize)*cos(angle), 0.001);
		object->bound->setLocalPos((distance - stepsize)*sin(angle), (distance - stepsize)*cos(angle), 0.001);

	}
	
}


//------------------------------------------------------------------------------

void updateHaptics(void)
{
    // initialize frequency counter
    frequencyCounter.reset();

    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

	cPrecisionClock timer;
	timer.start();
	double t = timer.getCurrentTimeSeconds();
	double toolGCD = 1.5; // tool's general cooldown in seconds
	double enlargeCD = 10; // cooldown for enlarge the tool

	point * objectWeight = new point(mass, spring, springDamp, cVector3d(0, -0.0375, 0.05), spring1);

	objectWeightHolder->setLocalPos(objectWeight->anchor);



	point * particle1Wrapper = new point(0.5, 400.0, 1, cVector3d(0, -0.0375, 0.05), spring1);

	particle1->setLocalPos(particle1Wrapper->anchor);


    // main haptic simulation loop
    while(simulationRunning)
    {



		// timer
		double t = timer.getCurrentTimeSeconds();
		

		if ((t - previousT) > 0.1){
			previousT = t;
			for (int i = 0; i < 8; i++){
				calcAntPos(minions[i]);
			}

			if (t > 5.5){
				for (int i = 8; i < 16; i++){
					calcAntPos(minions[i]);
				}
			}
		}

		//cVector3d myPos = tool->getLocalPos();
		//if (myPos.x() < (-0.05 + 0.003) || myPos.x() > (-0.05 - 0.003)){
		//	if (myPos.y() < (0.05 + 0.003) || myPos.y() > (0.05 - 0.003)){

		//	}
		//}


        /////////////////////////////////////////////////////////////////////
        // READ HAPTIC DEVICE
        /////////////////////////////////////////////////////////////////////

        // read position 
        cVector3d position;
        hapticDevice->getPosition(position);
		//position = tool->getLocalPos();

        // read orientation 
        cMatrix3d rotation;
        hapticDevice->getRotation(rotation);

        // read user-switch status (button 0)
		bool mainButton = false;
		hapticDevice->getUserSwitch(0, mainButton);

		bool leftButton = false;
		hapticDevice->getUserSwitch(1, leftButton);

		bool rightButton = false;
		hapticDevice->getUserSwitch(3, rightButton);


		/////////////////////////////////////////////////////////////////////
		// UPDATE 3D CURSOR MODEL
		/////////////////////////////////////////////////////////////////////

		// update position and orienation of cursor
		//cursor->setLocalPos(position);
		//cursor->setLocalRot(rotation);



		

		/////////////////////////////////////////////////////////////////////
		// COMPUTE FORCES
		/////////////////////////////////////////////////////////////////////


		
		double freq = 25.0; //Hz
		double y_s = 0.04 * sin(2.0 * C_PI * freq * t);
		double delta_t = timer.getCurrentTimeSeconds() - t;
		t += delta_t;


		cHapticPoint* interactionPoint = tool->getHapticPoint(0);
		world->computeGlobalPositions();

		int x;

		//cVector3d force(0, 0, 0);

		// check primary contact point if available
		if (holding == 1)
		{
			/*if (position.z() - 5 * toolRadius < 0)
				held->mesh->setLocalPos(cVector3d(position.x(), position.y(), 0));
			else*/
			held->mesh->setLocalPos(cVector3d(position.x(), position.y(), position.z()));
				//cout << position.z() - 3 * toolRadius << endl;
			//objectWeightHolder->setLocalPos(position);
			/*objectWeight->anchor = position;
			objectWeight->computeSpringForces(position, objectWeightHolder, delta_t);

			spring1->m_pointA = position;
			spring1->m_pointB = held->mesh->getLocalPos();

			objectWeightHolder->setLocalPos(objectWeight->pposition);*/


			particle1Wrapper->anchor = position;

			particle1Wrapper->computeSpringForces(position, particle1, delta_t);

			// set particle's position

			spring1->m_pointA = particle1Wrapper->anchor;
			spring1->m_pointB = particle1Wrapper->pposition;

			particle1->setLocalPos(particle1Wrapper->pposition);

		}
		else
		{

			

			if (touched == false && holding == 0)
			{
				force = cVector3d(0, 0, 0);
			}



			if (interactionPoint->getNumCollisionEvents() > 0 && holding == 0 && touched == true)
			{

				cCollisionEvent* collisionEvent = interactionPoint->getCollisionEvent(0);

				cGenericObject* object = collisionEvent->m_object->getOwner();

				cMultiMesh* multiObject = dynamic_cast<cMultiMesh*>(object);

				if (counter < 30 && object != table && object != cakeBox)
				{
					cVector3d setpoint(0, y_s, 0);
					force = force + (-150 * (position - setpoint));
					torque = cVector3d(0, 0, 0);
					gripperForce = 0.0;
					counter++;
				}
				else
					force = cVector3d(0, 0, 0);
			}


			if (interactionPoint->getNumCollisionEvents() > 0 && touched == false && holding == 0)
			{
				
				cCollisionEvent* collisionEvent = interactionPoint->getCollisionEvent(0);

				cGenericObject* object = collisionEvent->m_object->getOwner();

				cMultiMesh* multiObject = dynamic_cast<cMultiMesh*>(object);




				for (int i = 0; i < minions.size(); i++)
				{
					if (minions[i]->mesh == multiObject)
					{
						x = i;
						//cout << "hit " << x << endl;
						cVector3d centre = minions[x]->mesh->getLocalPos();

						if (abs(position.z()) > abs(minions[x]->mesh->getLocalPos().z()))
						{
							cout << "position: " << abs(position.z()) << endl;
							cout << "mesh: " << abs(minions[x]->mesh->getLocalPos().z()) << endl;

							if (abs(position.z()) < abs(minions[x]->mesh->getLocalPos().z()) + topOffset && abs(position.z()) > abs(minions[x]->mesh->getLocalPos().z()+ bottomOffset))
							{
								minions[x]->hitpoints--;
								cout << "hit " << x << endl;


								hit = true;
								touched = true;
								if (minions[x]->hitpoints <= 0)
								{

									minions[x]->bound->setEnabled(0);
									minions[x]->mesh->setEnabled(0);
									//world->removeChild(object);
									touched = false;
									if (minions[x]->type == "normal")
										money++;
									else if (minions[x]->type == "quick")
										money = money + 2;
								}
							}
						}
					}
				}

				for (int i = 0; i < objects.size(); i++)
				{
					if (objects[i]->mesh == multiObject && mainButton == true)
					{
						x = i;
						holding = 1;
						pushed = 2;
						held = objects[i];
						//objectWeight = new point(mass, spring, springDamp, position, spring1);
						//objectWeightHolder->setLocalPos(position);
						/*objectWeight->anchor = position;
						objectWeight->computeSpringForces(position, objectWeightHolder, delta_t);

						spring1->m_pointA = objectWeight->anchor;
						spring1->m_pointB = objectWeight->pposition;

						objectWeightHolder->setLocalPos(objectWeight->pposition);*/
						

						particle1Wrapper->anchor = position;

						particle1Wrapper->computeSpringForces(position, particle1, delta_t);

						// set particle's position

						spring1->m_pointA = particle1Wrapper->anchor;
						spring1->m_pointB = particle1Wrapper->pposition;

						particle1->setLocalPos(particle1Wrapper->pposition);

						//cout << "Picked up";
					}
				}


			}

			if (interactionPoint->getNumCollisionEvents() <= 0 && holding == 0)
			{
				hit = false;
				touched = false;
				force = cVector3d(0, 0, 0);
				counter = 0;
			}
		}


		if (mainButton == TRUE && pushed == 0)
			pushed = 1;

		if (pushed == 1 && holding == 0)
		{
			holding = 1;
			force = cVector3d(0, 0, -10);

			unit *first = new unit;
			held = first;
			monkey = new cMultiMesh();
			first->mesh = monkey;
			first->hitpoints = 50;

			objects.push_back(first);

			world->addChild(objects[objects.size() - 1]->mesh);


			objects[objects.size() - 1]->mesh->loadFromFile("whirling_blades.obj");

			/*if (position.z() - 5 * toolRadius < 0)
				objects[objects.size() - 1]->mesh->setLocalPos(cVector3d(position.x(), position.y(), 0));
			else*/
				objects[objects.size() - 1]->mesh->setLocalPos(cVector3d(position.x(), position.y(), 0.001));

			objects[objects.size() - 1]->mesh->scale(0.00325);
			first->type = "quick";

			changeCollisionDetector("off");

			//objectWeight = new point(mass, spring, springDamp, position, spring1);
			//objectWeightHolder->setLocalPos(position);
			/*objectWeight->anchor = position;
			objectWeight->computeSpringForces(position, objectWeightHolder, delta_t);

			spring1->m_pointA = objectWeight->anchor;
			spring1->m_pointB = objectWeight->pposition;

			objectWeightHolder->setLocalPos(objectWeight->pposition);*/


			particle1Wrapper->anchor = position;

			particle1Wrapper->computeSpringForces(position, particle1, delta_t);

			// set particle's position

			spring1->m_pointA = particle1Wrapper->anchor;
			spring1->m_pointB = particle1Wrapper->pposition;

			particle1->setLocalPos(particle1Wrapper->pposition);

			pushed = 2;

		}

		else if (pushed == 1 && holding == 1)
		{
			holding = 0;
			held = NULL;
			changeCollisionDetector("on");
			objects[objects.size() - 1]->mesh->computeBoundaryBox(true);
			objects[objects.size() - 1]->mesh->createAABBCollisionDetector(toolRadius);
			objects[objects.size() - 1]->mesh->setStiffness(500, true);

			objects[objects.size() - 1]->mesh->setFriction(0.1, 0.2, true);
			pushed = 2;
		}

		if (leftButton == TRUE && held != NULL)
		{
			held->mesh->rotateAboutLocalAxisDeg(cVector3d(0, 0, 1), 0.5);
		}

		else if (rightButton == TRUE && held != NULL)
		{
			held->mesh->rotateAboutLocalAxisDeg(cVector3d(0, 0, 1), -0.5);
		}

		if (mainButton == FALSE && pushed == 2)
			pushed = 0;


		/////////////////////////////////////////////////////////////////////
		// APPLY FORCES
		/////////////////////////////////////////////////////////////////////
		//changeCollisionDetector("off");
		//force = cVector3d(0, 0, -500);

		// send computed force, torque, and gripper force to haptic device
		//if (!force.equals(cVector3d(0, 0, 0)))
			//tool->addDeviceGlobalForce(force);
			//hapticDevice->setForceAndTorqueAndGripperForce(force, torque, gripperForce);


			tool->updateFromDevice();
			tool->computeInteractionForces();


			tool->addDeviceGlobalForce(force);

			tool->applyToDevice();

        // update frequency counter
        frequencyCounter.signal(1);
    }
    
    // exit haptics thread
    simulationFinished = true;
}

//------------------------------------------------------------------------------
