
#if defined(_WIN32)
#define Bullet3Lua_API  extern "C" __declspec(dllexport)
#else
#define Bullet3Lua_API		extern "C"
#endif

#include "../CommonInterfaces/CommonExampleInterface.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "../Utils/b3Clock.h"

#include "../OpenGLWindow/SimpleOpenGL2App.h"
#include "../OpenGLWindow/SimpleOpenGL2Renderer.h"
#include <stdio.h>
#include "../ExampleBrowser/OpenGLGuiHelper.h"
#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../Extras/ConvexDecomposition/cd_wavefront.h"
#include "../src/BulletCollision/Gimpact/btGImpactShape.h"

extern "C" {
#include "lua.h" 
#include "lualib.h" 
#include "lauxlib.h" 
}

static CommonRigidBodyBase* example = 0;
static SimpleOpenGL2App* app = 0;
static b3Clock clock;

static b3MouseMoveCallback prevMouseMoveCallback = 0;
static void OnMouseMove(float x, float y)
{
	bool handled = false;
	handled = example->mouseMoveCallback(x, y);
	if (!handled)
	{
		if (prevMouseMoveCallback)
			prevMouseMoveCallback(x, y);
	}
}

static b3MouseButtonCallback prevMouseButtonCallback = 0;
static void OnMouseDown(int button, int state, float x, float y)
{
	bool handled = false;

	handled = example->mouseButtonCallback(button, state, x, y);
	if (!handled)
	{
		if (prevMouseButtonCallback)
			prevMouseButtonCallback(button, state, x, y);
	}
}

class LessDummyGuiHelper : public DummyGUIHelper
{
	CommonGraphicsApp* m_app;

public:
	virtual CommonGraphicsApp* getAppInterface()
	{
		return m_app;
	}

	LessDummyGuiHelper(CommonGraphicsApp* app)
		: m_app(app)
	{
	}
};

static int gui_update() {

	app->m_renderer->init();
	app->m_renderer->updateCamera(app->getUpAxis());

	btScalar dtSec = btScalar(clock.getTimeInSeconds());

	example->stepSimulation(dtSec);
	clock.reset();

	example->renderScene();

	DrawGridData dg;
	dg.upAxis = app->getUpAxis();
	app->drawGrid(dg);

	app->swapBuffer();

	if (app->m_window->requestedExit()) {
		example->exitPhysics();
		delete example;
		delete app;
		exit(0);
		return 0;
	} else {
		return 1;
	}
}

static void gui_init(int width, int height)
{
	if (example != 0) {
		example->exitPhysics();
		delete example;
	}

	if (app != 0) {
		delete app;
	}

	app = new SimpleOpenGL2App("Bullet Standalone Example", width, height);
	app->m_renderer = new SimpleOpenGL2Renderer(width, height);

	prevMouseButtonCallback = app->m_window->getMouseButtonCallback();
	prevMouseMoveCallback = app->m_window->getMouseMoveCallback();

	app->m_window->setMouseButtonCallback((b3MouseButtonCallback)OnMouseDown);
	app->m_window->setMouseMoveCallback((b3MouseMoveCallback)OnMouseMove);

	OpenGLGuiHelper* gui = new OpenGLGuiHelper(app, false);
	CommonExampleOptions* options = new CommonExampleOptions(gui);

	example = (CommonRigidBodyBase*)StandaloneExampleCreateFunc(*options);

	example->initPhysics();
	example->resetCamera();
}

static int nogui_update() {

	b3Clock clock;
	btScalar dtSec = btScalar(clock.getTimeInSeconds());

	example->stepSimulation(dtSec);

	return 0;
}

static void nogui_init(int width, int height)
{
	if (example != 0) {
		example->exitPhysics();
		delete example;
	}

	if (app != 0) {
		delete app;
	}

	DummyGUIHelper noGfx;
	CommonExampleOptions options(&noGfx);
	example = (CommonRigidBodyBase*)StandaloneExampleCreateFunc(options);

	example->initPhysics();
	example->resetCamera();
}

static void destory()
{
	if (example != 0) {
		example->exitPhysics();
		delete example;
	}

	if (app != 0) {
		delete app;
	}
}

static int bullet3_guiupdate(lua_State *L){
	int r = gui_update();
	lua_pushinteger(L, r);
	return 1;
}

static int bullet3_guiinit(lua_State *L){

	int width = luaL_checkinteger(L, 1);
	int height = luaL_checkinteger(L, 2);
	gui_init(width, height);
	return 0;
}

static int bullet3_destory(lua_State *L){
	destory();
	return 0;
}

static int bullet3_update(lua_State *L){
	int r = nogui_update();
	lua_pushinteger(L, r);
	return 1;
}

static int bullet3_init(lua_State *L){

	int width = luaL_checkinteger(L, 1);
	int height = luaL_checkinteger(L, 2);
	nogui_init(width, height);
	return 0;
}

static int bullet3_loadOBJ(lua_State *L){

	const char* file = luaL_checkstring(L, 1);
	float x = luaL_checknumber(L, 2);
	float y = luaL_checknumber(L, 3);
	float z = luaL_checknumber(L, 4);

	float Roll = luaL_checknumber(L, 5);
	float Pitch = luaL_checknumber(L, 6);
	float Yaw = luaL_checknumber(L, 7);

	float mass = luaL_checknumber(L, 8);

	ConvexDecomposition::WavefrontObj wobj;
	std::string filename(file);
	int result = wobj.loadObj(file);
	if (!result) {
		return 0;
	}

	printf("--load status %d\n", result);
	printf("--triangle: %d\n", wobj.mTriCount);
	printf("--vertex: %d\n", wobj.mVertexCount);

	btScalar Vertices = *wobj.mVertices;
	btTriangleIndexVertexArray* colonVertexArrays = new btTriangleIndexVertexArray(
		wobj.mTriCount,
		wobj.mIndices,
		3 * sizeof(int),
		wobj.mVertexCount,
		&Vertices,
		3 * sizeof(float)
		);

	btGImpactMeshShape* bunnymesh = new btGImpactMeshShape(colonVertexArrays);
	bunnymesh->setLocalScaling(btVector3(0.5f, 0.5f, 0.5f));
	bunnymesh->updateBound();

	btTransform startTransform;
	startTransform.setOrigin(btVector3(x, y, z));
	startTransform.getBasis().setEulerZYX(Roll, Pitch, Yaw);

	example->createRigidBody(mass, startTransform, bunnymesh);
	return 0;
}

void bullet3_eventFunCall(void* character, void* m_param, btKinematicCharacterController::EVENT_CONTROL ec, btVector3& start, btVector3& end, btScalar angle) {

	btKinematicCharacterController* btCharacter = (btKinematicCharacterController*)character;
	lua_State *L = (lua_State *)m_param;
	lua_getglobal(L, btCharacter->callFun.c_str());

	lua_newtable(L);
	lua_pushstring(L, "name");
	lua_pushstring(L, btCharacter->strName.m_string1.c_str());
	lua_settable(L, -3);

	lua_pushstring(L, "control");
	lua_pushinteger(L, ec);
	lua_settable(L, -3);

	lua_pushstring(L, "start_x");
	lua_pushnumber(L, start.getX());
	lua_settable(L, -3);

	lua_pushstring(L, "start_y");
	lua_pushnumber(L, start.getY());
	lua_settable(L, -3);

	lua_pushstring(L, "start_z");
	lua_pushnumber(L, start.getZ());
	lua_settable(L, -3);

	lua_pushstring(L, "end_x");
	lua_pushnumber(L, end.getX());
	lua_settable(L, -3);

	lua_pushstring(L, "end_y");
	lua_pushnumber(L, end.getY());
	lua_settable(L, -3);

	lua_pushstring(L, "end_z");
	lua_pushnumber(L, end.getZ());
	lua_settable(L, -3);

	lua_pushstring(L, "angle");
	lua_pushnumber(L, angle);
	lua_settable(L, -3);

	lua_pcall(L, 1, LUA_MULTRET, 0);
	lua_settop(L, 0);
}

static int bullet3_addCharacter(lua_State *L){
	btHashString name = luaL_checkstring(L, 1);

	float x = luaL_checknumber(L, 2);
	float y = luaL_checknumber(L, 3);
	float z = luaL_checknumber(L, 4);

	std::string callFun = luaL_checkstring(L, 5);
	example->addCharacter(name, L, btVector3(x, y, z), bullet3_eventFunCall, callFun);
	return 0;
}

static int bullet3_addCharacter2(lua_State *L){
	btHashString name = luaL_checkstring(L, 1);

	float x = luaL_checknumber(L, 2);
	float y = luaL_checknumber(L, 3);
	float z = luaL_checknumber(L, 4);

	std::string callFun = luaL_checkstring(L, 5);

	float height = luaL_checknumber(L, 6);
	float weidth = luaL_checknumber(L, 7);
	float stepHeight = luaL_checknumber(L, 8);

	int useGhostObjectSweepTest = luaL_checkinteger(L, 9);
	int collisionFilterGroup = luaL_checkinteger(L, 10);
	int collisionFilterMask = luaL_checkinteger(L, 11);

	example->addCharacter(name, L, btVector3(x, y, z), bullet3_eventFunCall, callFun, height, weidth, stepHeight, (bool)useGhostObjectSweepTest, collisionFilterGroup, collisionFilterMask);
	return 0;
}

static int bullet3_removeCharacter(lua_State *L){
	btHashString name = luaL_checkstring(L, 1);

	example->removeCharacter(name);
	return 0;
}

static int bullet3_moveDir(lua_State *L){
	btHashString name = luaL_checkstring(L, 1);

	float x = luaL_checknumber(L, 2);
	float y = luaL_checknumber(L, 3);
	float z = luaL_checknumber(L, 4);

	example->move(name, btVector3(x, y, z));
	return 0;
}

static int bullet3_moveName(lua_State *L){
	btHashString name = luaL_checkstring(L, 1);
	btHashString name2 = luaL_checkstring(L, 2);

	btPairCachingGhostObject** ghostObject2 = example->m_ghostObject.find(name2);
	if (ghostObject2) {
		btTransform& bt = (*ghostObject2)->getWorldTransform();
		example->move(name, bt.getOrigin());
	}
	
	return 0;
}

static int bullet3_moveKey(lua_State *L){
	btHashString name = luaL_checkstring(L, 1);
	unsigned int key = luaL_checkinteger(L, 2);

	example->move(name, key);
	return 0;
}

static int bullet3_rayTest(lua_State *L){

	float from_x = luaL_checknumber(L, 1);
	float from_y = luaL_checknumber(L, 2);
	float from_z = luaL_checknumber(L, 3);

	float to_x = luaL_checknumber(L, 4);
	float to_y = luaL_checknumber(L, 5);
	float to_z = luaL_checknumber(L, 6);

	btVector3 p;
	std::string name;
	int r = example->rayCharacter(btVector3(from_x, from_y, from_z), btVector3(to_x, to_y, to_z), p, name);

	lua_createtable(L, 0, 0);

	lua_pushstring(L, "isRay");
	lua_pushnumber(L, r);
	lua_settable(L, -3);

	lua_pushstring(L, "x");
	lua_pushnumber(L, p.getX());
	lua_settable(L, -3);

	lua_pushstring(L, "y");
	lua_pushnumber(L, p.getY());
	lua_settable(L, -3);

	lua_pushstring(L, "z");
	lua_pushnumber(L, p.getZ());
	lua_settable(L, -3);

	lua_pushstring(L, "name");
	lua_pushstring(L, name.c_str());
	lua_settable(L, -3);

	return 1;
}

static int bullet3_addBoxShape(lua_State *L) {
	btHashString name = luaL_checkstring(L, 1);

	float x = luaL_checknumber(L, 2);
	float y = luaL_checknumber(L, 3);
	float z = luaL_checknumber(L, 4);

	float ox = luaL_checknumber(L, 5);
	float oy = luaL_checknumber(L, 6);
	float oz = luaL_checknumber(L, 7);

	float Roll = luaL_checknumber(L, 8);
	float Pitch = luaL_checknumber(L, 9);
	float Yaw = luaL_checknumber(L, 10);

	float mass = luaL_checknumber(L, 11);

	///create a few basic rigid bodies
	btBoxShape* groundShape = example->createBoxShape(btVector3(btScalar(ox), btScalar(oy), btScalar(oz)));
	example->m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(x, y, z));
	groundTransform.getBasis().setEulerZYX(Roll, Pitch, Yaw);

	{
		btRigidBody* rigidBody = example->createRigidBody(mass, groundTransform, groundShape, btVector4(0, 0, 1, 1));
		example->m_btRigidBody.insert(name, rigidBody);
	}
	
	return 0;

}

static int bullet3_GraphicsObjects(lua_State *L) {
	example->m_guiHelper->autogenerateGraphicsObjects(example->m_dynamicsWorld);
	return 0;
}

static int bullet3_getTransform(lua_State *L){
	btHashString name = luaL_checkstring(L, 1);
	btPairCachingGhostObject** ghostObject= example->m_ghostObject.find(name);
	if (*ghostObject) {
		btTransform& bt = (*ghostObject)->getWorldTransform();

		lua_createtable(L, 0, 0);

		lua_pushstring(L, "x");
		lua_pushnumber(L, bt.getOrigin().getX());
		lua_settable(L, -3);

		lua_pushstring(L, "y");
		lua_pushnumber(L, bt.getOrigin().getY());
		lua_settable(L, -3);

		lua_pushstring(L, "z");
		lua_pushnumber(L, bt.getOrigin().getZ());
		lua_settable(L, -3);
		return 1;
	} else {
		lua_pushnil(L);
		return 1;
	}
}

static int bullet3_getDistance(lua_State *L){

	btHashString name = luaL_checkstring(L, 1);
	btHashString name2 = luaL_checkstring(L, 2);

	btPairCachingGhostObject** ghostObject = example->m_ghostObject.find(name);
	btPairCachingGhostObject** ghostObject2 = example->m_ghostObject.find(name2);
	if (ghostObject && ghostObject2) {
		btTransform& bt = (*ghostObject)->getWorldTransform();
		btTransform& bt2 = (*ghostObject2)->getWorldTransform();
		lua_pushnumber(L, bt.getOrigin().distance2(bt2.getOrigin()));
		return 1;
	} else {
		lua_pushnil(L);
		return 1;
	}
}

static int bullet3_deleteRigidBody(lua_State *L){
	btHashString name = luaL_checkstring(L, 1);
	btRigidBody** rigidBody = example->m_btRigidBody.find(name);
	if (*rigidBody) {
		example->deleteRigidBody(*rigidBody);
		example->m_btRigidBody.remove(name);
	}

	return 0;
}

static const luaL_reg Mylib[] = {

	{ "loadOBJ", bullet3_loadOBJ },
	{ "boxShape", bullet3_addBoxShape },
	{ "deleteRigidBody", bullet3_deleteRigidBody },
	
	{ "character", bullet3_addCharacter },
	{ "character2", bullet3_addCharacter2 },
	{ "removeCharacter", bullet3_removeCharacter },

	{ "moveName", bullet3_moveName },
	{ "moveDir", bullet3_moveDir },
	{ "moveKey", bullet3_moveKey },
	{ "rayTest", bullet3_rayTest},

	{ "update", bullet3_update },
	{ "init", bullet3_init },
	

	{ "gui_update", bullet3_guiupdate },
	{ "gui_init", bullet3_guiinit },
	
	{ "graphicsObjects", bullet3_GraphicsObjects },
	{ "getDistance", bullet3_getDistance },
	{ "getTransform", bullet3_getTransform },
	{ "destroy", bullet3_destory },
	{ NULL, NULL },
};

Bullet3Lua_API int luaopen_Bullet3LuaWarp(lua_State *L)
{
	luaL_register(L, "Bullet3LuaWarp", Mylib);
	return 1;
}