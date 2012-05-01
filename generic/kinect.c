#ifndef TH_GENERIC_FILE
#define TH_GENERIC_FILE "generic/kinect.c"
#else

//======================================================================
// File: kinect
//
// Description: A wrapper for libfreenect
//
// Created: May  1, 2012, 12:25PM
//
// Install:
// http://openkinect.org/wiki/Getting_Started
//
// Author: Benoit Corda // cordaben@gmail.com
//======================================================================

#include <luaT.h>
#include <TH.h>


//===========================================================
// generic functions

/*******************
 grad the rgb frame
*******************/
static int libkinect_(grab_rgb) (lua_State *L) {
  // Get Tensor's Info
  THTensor * tensor = luaT_checkudata(L, 1, torch_(Tensor_id));
  THTensor *contigTensor = THTensor_(newContiguous)(tensor);
  // Get device ID
  int index = 0;
  if (lua_isnumber(L, 2)) index = lua_tonumber(L, 2);


  THArgCheck(tensor->size[0] == 3 , 1, "RBG buffer: 3x480x640 Tensor expected");
  THArgCheck(tensor->size[1] == 480 , 1, "RBG buffer: 3x480x640 Tensor expected");
  THArgCheck(tensor->size[2] == 640 , 1, "RBG buffer: 3x480x640 Tensor expected");

  unsigned int timestamp;
  unsigned char *data = 0;
  if (freenect_sync_get_video((void**)&data, &timestamp, index, FREENECT_VIDEO_RGB))
    luaL_error(L, "<libkinect.grabRGB> Error Kinect not connected?");

  int z;
  for (z=0;z<3;z++){
    unsigned char *sourcep = data+z;
    THTensor *tslice = THTensor_(newSelect)(contigTensor,0,z);
    // copy
    TH_TENSOR_APPLY(real, tslice,
        	    *tslice_data = ((real)(*sourcep));
        	    sourcep = sourcep + 3;
        	    );
    THTensor_(free)(tslice);
  }

  // return the timestamp
  lua_pushnumber(L, timestamp);

  return 1;
}

//============================================================
// Register functions in LUA
//
static const luaL_reg libkinect_(Main__) [] =
{
  {"grabRGB", libkinect_(grab_rgb)},
  {NULL, NULL}  /* sentinel */
};

DLL_EXPORT int libkinect_(Main_init) (lua_State *L) {
  luaT_pushmetaclass(L, torch_(Tensor_id));
  luaT_registeratname(L, libkinect_(Main__), "libkinect");
  return 1;
}

#endif
