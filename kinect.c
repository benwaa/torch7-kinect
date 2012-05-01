/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2010 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 *
 * Lua Wrapper Written by: Benoit Corda // cordaben@gmail.com
 * Created: May  1, 2012, 9:25AM
 */

#include <luaT.h>
#include <TH.h>

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "libfreenect_sync.h"

#include <pthread.h>

#include <math.h>
#define max(a,b) a < b ? b : a
#define min(a,b) a > b ? b : a

#define FREENECT_FRAME_W 480
#define FREENECT_FRAME_H 640
#define VFORMAT FREENECT_VIDEO_RGB
#define DFORMAT FREENECT_DEPTH_11BIT
#define D_MAXSIZE 2047


#define torch_(NAME) TH_CONCAT_3(torch_, Real, NAME)
#define torch_string_(NAME) TH_CONCAT_STRING_3(torch., Real, NAME)
#define libkinect_(NAME) TH_CONCAT_3(libkinect_, Real, NAME)

static const void* torch_FloatTensor_id = NULL;
static const void* torch_DoubleTensor_id = NULL;

#include "generic/kinect.c"
#include "THGenerateFloatTypes.h"

/*
static int l_grab_depth(lua_State *L) {
  THTensor * tensor = NULL;
  if (luaT_isudata(L, 1, luaT_checktypename2id(L, "torch.Tensor"))) {
    tensor = luaT_toudata(L, 1, luaT_checktypename2id(L, "torch.Tensor"));
  } else {
    perror("getDepth: tensor should be double");
    lua_pushnil(L);  // return nil and ...
    return 1;
  }

  unsigned int timestamp;
  short *data = 0;
  int index = 0;
  int height,width;
  if (freenect_sync_get_depth((void**)&data, &timestamp, index, DFORMAT))
    luaL_error(L, "<libkinect.grabDepth> Error Kinect not connected?");

  int h,w,z;
  double ma = 0;
  width = min(tensor->size[0], FREENECT_FRAME_W);
  height = min(tensor->size[1], FREENECT_FRAME_H);
  if (tensor->nDimension == 2) {
    for (h=0; h<height; h++)
      for (w=0; w<width; w++)
        THTensor_set2d(tensor, w, h, (double)data[(h*FREENECT_FRAME_W+w) / D_MAXSIZE]);
  } else if (tensor->nDimension == 3) {
    if (tensor->size[2] == 1) {
      for (h=0; h<height; h++)
        for (w=0; w<width; w++)
          THTensor_set3d(tensor, w, h, 0, (double)data[(h*FREENECT_FRAME_W+w)] / D_MAXSIZE);
    } else {
      luaL_error(L, "<libkinect.grabDepth> Tensor must be 2D or 3rd dim equal 1");
    }
  }

  return 0;
}
*/

/*
static int l_grab_rgbD(lua_State *L) {
  THTensor * tensor = NULL;
  if (luaT_isudata(L, 1, luaT_checktypename2id(L, "torch.Tensor"))) {
    tensor = luaT_toudata(L, 1, luaT_checktypename2id(L, "torch.Tensor"));
  } else {
    perror("getRGBD: tensor should be double");
    lua_pushnil(L);  // return nil and ...
    return 1;
  }
  if (tensor->nDimension != 3 || tensor->size[2] != 4)
    luaL_error(L, "<libkinect.grabRgbD> Error the Tensor must be 3D with 4 channels in the 3rd Dim");

  int index = 0;
  unsigned int timestamp;
  int height,width;

  width = min(tensor->size[0], FREENECT_FRAME_W);
  height = min(tensor->size[1], FREENECT_FRAME_H);

  // copy the rgb channels
  unsigned char *vid = 0;
  if (freenect_sync_get_video((void**)&vid, &timestamp, index, FREENECT_VIDEO_RGB))
    luaL_error(L, "<libkinect.grabRgb> Error Kinect not connected?");
  int h,w,z;
  for (h=0; h<height; h++)
    for (w=0; w<width; w++)
      for (z=0; z<3; z++) {
        double pix = ((double)vid[(h*FREENECT_FRAME_W+w)*3+z]) / 255;
        THTensor_set3d(tensor, w, h, z, pix);
      }

  // copy depth
  uint16_t *depth = 0;
  if (freenect_sync_get_depth((void**)&depth, &timestamp, index, DFORMAT))
    luaL_error(L, "<libkinect.grabDepth> Error Kinect not connected?");
  // copy the depth channel
  for (h=0; h<height; h++)
    for (w=0; w<width; w++){
      uint16_t val = depth[(h*FREENECT_FRAME_W+w)];
      THTensor_set3d(tensor, w, h, 3, (double)val / D_MAXSIZE);
    }

  return 0;
}
*/


/******************************
 userdata to impose gc on exit
******************************/
typedef struct l_userdata {
  int index;
  bool ison;    /* to know if the kinect is on*/
  int led;
} l_userdata;

/**************************************************
 init device and start the thread for grabbing
**************************************************/
static int l_init_kinect(lua_State * L){
  int index = 0;
  if (lua_isnumber(L, 1)) index = lua_tonumber(L, 1);

  // create a kinect object
  l_userdata *kinect = (l_userdata *)lua_newuserdata(L, sizeof(l_userdata));

  // set its metatable
  luaL_getmetatable(L, "libkinect");
  lua_setmetatable(L, -2);

  // init the device
  char buff[255];
  sprintf(buff, "Init Kinect ID #%d failed, did you plug the device?",index);
  if (wrap_setup_kinect(index, VFORMAT, 0))
    luaL_error(L, buff);
  kinect->index = index;
  kinect->ison = true;
  printf("Init Kinect ID #%d done...\n",index);

  return 1;
}


/******************************
 turn off the flag to avoid gc
******************************/
static int l_led(lua_State *L) {
  l_userdata *kinect = lua_touserdata(L, 2);
  // get args
  if (lua_isnumber(L, 1)) kinect->led = lua_tonumber(L, 1);
  else
    luaL_error(L, "<libkinect.tilt> you need to provide a tilt value in range [-30,30]");
  freenect_sync_set_led(kinect->led,kinect->index);
  return 0;
}


/****************
 tilt the kinect
****************/
static int l_tilt(lua_State *L) {
  int angle = 0;
  int index = 0;
  // get args
  if (lua_isnumber(L, 1)) angle = lua_tonumber(L, 1);
  else
    luaL_error(L, "<libkinect.tilt> you need to provide a tilt value in range [-30,30]");
  if (lua_isnumber(L, 2)) index = lua_tonumber(L, 2);

  // set the proper range
  if (angle > 30) {
    printf("<libkinect.tilt> tilt can't be more than 30\n");
    angle = 30;
  } else if (angle < -30) {
    printf("<libkinect.tilt> tilt must be at least 0\n");
    angle = 0;
  }
  freenect_sync_set_tilt_degs(angle, index);
  return 0;
}

/******************************
 turn off the flag to avoid gc
******************************/
static int l_off(lua_State *L) {
  l_userdata *kinect = lua_touserdata(L, 1);
  printf("Turning Kinect ID #%d off...\n", kinect->index);
  kinect->ison = false;
  kinect->led = 0;
  freenect_sync_set_led(kinect->led,kinect->index);
  return 0;
}


/******************************
 stop the global thread
******************************/
static int l_stop(lua_State *L) {
  freenect_sync_stop();
  return 0;
}

/************************************************
 garbage collection: stop the thread if needed
************************************************/
static int l_gc(lua_State *L) {
  l_userdata *kinect = lua_touserdata(L, 1);
  // check if kinect is still on
  if (kinect->ison){
    printf("Turning Kinect ID #%d off...\n", kinect->index);
    kinect->ison = false;
    kinect->led = 0;
    freenect_sync_set_led(kinect->led,kinect->index);
    freenect_sync_stop();
  }
  return 0;
}

static int l_tostring (lua_State *L)
{
  l_userdata *kinect = lua_touserdata(L, 1);
  lua_pushfstring(L, "Kinect %d is %s.\n",kinect->index,kinect->ison?"ON":"OFF");
  return 1;
}

/******************************
 userdata to impose gc on exit
******************************/
static const luaL_reg Kinect_meta[] = {
  {"__gc",       l_gc},
  {"__tostring", l_tostring},
  {NULL, NULL}  /* sentinel */
};


/*******************
 Register functions
*******************/
static const struct luaL_reg kinect [] = {
  {"newdevice", l_init_kinect},
  {"led", l_led},
  {"tilt", l_tilt},
  {"turnOff", l_off},
  {"stop", l_stop},
  {NULL, NULL}  /* sentinel */
};

DLL_EXPORT int luaopen_libkinect(lua_State * L) {
  luaL_openlib(L, "libkinect", kinect, 0);
  /* create metatable */
  luaL_newmetatable(L, "libkinect");
  /* fill metatable */
  luaL_openlib(L, 0, Kinect_meta, 0);
  lua_pushliteral(L, "__index");
  lua_pushvalue(L, -3);   /* dup methods table*/
  lua_rawset(L, -3);      /* metatable.__index = methods */
  lua_pushliteral(L, "__metatable");
  lua_pushvalue(L, -3);   /* dup methods table*/
  lua_rawset(L, -3);      /* hide metatable:
                             metatable.__metatable = methods */
  lua_pop(L, 1);         /* drop metatable */

  torch_FloatTensor_id = luaT_checktypename2id(L, "torch.FloatTensor");
  torch_DoubleTensor_id = luaT_checktypename2id(L, "torch.DoubleTensor");

  libkinect_FloatMain_init(L);
  libkinect_DoubleMain_init(L);

  luaL_register(L, "libkinect.double", libkinect_DoubleMain__);
  luaL_register(L, "libkinect.float", libkinect_FloatMain__);


  return 1;
}
