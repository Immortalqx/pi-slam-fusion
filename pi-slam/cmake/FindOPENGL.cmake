set(OPENGL_FOUND 1)

if( WIN32)     
    #set(OPENGL_LIBRARIES opengl32 glu32 freeglut glew32)
    set(OPENGL_LIBRARIES opengl32 glu32 freeglut_static)
    
    # FIXME: need better way to set lib path
    set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} /mingw64/lib)
else( WIN32 )
    set(OPENGL_LIBRARIES GL GLU glut X11)
endif( WIN32)

