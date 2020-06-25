
project "Bullet3LuaWarp"

if _OPTIONS["ios"] then
        kind "SharedLib"
else
        kind "SharedLib"
end
defines {"B3_USE_STANDALONE_EXAMPLE"}

includedirs {"../../src","./lua/"}

links {
        "BulletDynamics","BulletCollision", "LinearMath", "OpenGL_Window","Bullet3Common","lualib", "ConvexDecomposition"
}

  initOpenGL()
  initGlew()


language "C++"

files {
        "BasicExample.cpp",
	"Bullet3LuaWarp.cpp",
        "*.h",
        "../StandaloneMain/main_opengl_single_example.cpp",
	"../ExampleBrowser/OpenGLGuiHelper.cpp",
	"../ExampleBrowser/GL_ShapeDrawer.cpp",
	"../ExampleBrowser/CollisionShape2TriangleMesh.cpp",
	"../CommonInterfaces/*",
	"../Utils/b3Clock.cpp",
	"../Utils/b3Clock.h",
}

if os.is("Linux") then initX11() end

if os.is("MacOSX") then
        links{"Cocoa.framework"}
end
                          