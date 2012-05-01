-- This file is part of the OpenKinect Project. http://www.openkinect.org

-- Copyright (c) 2011 individual OpenKinect contributors. See the CONTRIB file
-- for details.

-- This code is licensed to you under the terms of the Apache License, version
-- 2.0, or, at your option, the terms of the GNU General Public License,
-- version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
-- or the following URLs:
-- http://www.apache.org/licenses/LICENSE-2.0
-- http://www.gnu.org/licenses/gpl-2.0.txt

-- If you redistribute this file in source form, modified or unmodified, you
-- may:
--   1) Leave this header intact and distribute it under the same terms,
--      accompanying it with the APACHE20 and GPL20 files, or
--   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
--   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
-- In all cases you must keep the copyright notice intact and include a copy
-- of the CONTRIB file.

-- Binary distributions must follow the binary distribution requirements of
-- either License.
--==============================================================================
-- File: init.lua
--
-- Description: A wrapper for using the kinect
--
--
-- Created: April  13, 2012, 4:55PM
--
-- Wrapper for Torch7/LUA: Benoit Corda -- cordaben@gmail.com
--==============================================================================
require 'torch'
require 'dok'
require 'image'
require 'libkinect'

kinect = {}
------------------------------------------------------------
-- local variables/function for internal handling
-- (context devices, etc...)
------------------------------------------------------------
local _kinect = {}
_kinect.devices = {}
_kinect.tensors = {}
-- list of colors
_kinect.colors = {off=0,green=1,red=2,orange=3,
                  green_wink=4,
                  orange_wink_red=6}
_kinect.current = nil -- current device in use
_kinect.grabbingColor = 6

function kinect.initDevice(...)
   local _,id = dok.unpack(
      {...},
      'kinect.device',
      [[return the current device from frame grabbing]],
      {arg='id', type='number', help='id of the device', default=0})
   if _kinect.devices[id] == nil then
      _kinect.devices[id] = libkinect.newdevice(id)
      _kinect.tensors[id] = {}
      -- set the led to show it's working
      _kinect.colors[id] = kinect.led{color='green',id=id}
      -- wait for a bit to avoid stall
      sys.sleep(0.3)
   end
   _kinect.current = id
   return _kinect.devices[id]
end

function kinect.getRGB(...)
   local _,id = dok.unpack(
      {...},
      'kinect.getRGB',
      [[return the current RGB frame, and its timestamp]],
      {arg='id', type='number', help='id of the device', default=_kinect.current})
   if id == nil then
      error("No Kinect ON, you need to initDevice first")
   end
   -- init tensor
   if _kinect.tensors[id].rgb == nil then
      _kinect.tensors[id].rgb = torch.Tensor(3,480,640)
   end
   -- set grabbing color to orange_wink_red
   if _kinect.colors[id] ~= _kinect.grabbingColor then
      _kinect.colors[id] = kinect.led{colorValue=_kinect.grabbingColor,id=id}
   end
   local rgb = _kinect.tensors[id].rgb
   -- c call
   local timestamp = rgb.libkinect.grabRGB(rgb,id)
   return rgb,timestamp
end

function kinect.getDepth(...)
   local _,id = dok.unpack(
      {...},
      'kinect.getDepth',
      [[return Depth map, timestamp ]],
      {arg='id', type='number', help='id of the device', default=_kinect.current})
   if id == nil then
      error("No Kinect ON, you need to initDevice first")
   end
   -- init tensor
   if _kinect.tensors[id].depth == nil then
      _kinect.tensors[id].depth = torch.Tensor(1,480,640)
   end
   -- set grabbing color to orange_wink_red
   if _kinect.colors[id] ~= _kinect.grabbingColor then
      _kinect.colors[id] = kinect.led{colorValue=_kinect.grabbingColor,id=id}
   end
   local depth = _kinect.tensors[id].depth
   -- c call
   local timestamp = depth.libkinect.grabDepth(depth,id)
   return depth, timestamp
end

function kinect.getRGBD(...)
   local _,id = dok.unpack(
      {...},
      'kinect.getRGBD',
      [[return RGBD maps, timestampRGB, timestampDepth ]],
      {arg='id', type='number', help='id of the device', default=_kinect.current})
   if id == nil then
      error("No Kinect ON, you need to initDevice first")
   end
   -- init tensor
   if _kinect.tensors[id].rgbd == nil then
      _kinect.tensors[id].rgbd = torch.Tensor(4,480,640)
   end
   -- set grabbing color to orange_wink_red
   if _kinect.colors[id] ~= _kinect.grabbingColor then
      _kinect.colors[id] = kinect.led{colorValue=_kinect.grabbingColor,id=id}
   end
   local rgbd = _kinect.tensors[id].rgbd
   -- c call
   local timestampRGB, timestampD = rgbd.libkinect.grabRGBD(rgbd,id)
   return rgbd, timestampRGB, timestampD
end

function kinect.led(...)
   local choices = ''
   for k,_ in pairs(_kinect.colors) do choices = choices .. ',"' .. k .. '"' end
   choices ='['..string.gsub(choices,"^,","") .. ']'
   local _,color,id,value = dok.unpack(
      {...},
      'kinect.led',
      [[change the LED color of the device]],
      {arg='color', type='number',
       help='new color for the LED among '..choices, default='orange_wink_red'},
      {arg='id', type='number', help='id of the device',
       default=_kinect.current},
      {arg='colorValue', type='number', help='new color numerical value'}
   )
   if id == nil then
      error("No Kinect ON, you need to initDevice first")
   end
   if not value then
      value = _kinect.colors[color]
   end
   if value == nil then
      error("Color unknow, choose among "..choices)
   end
   libkinect.led(value,_kinect.devices[id])
   return val
end


function kinect.tilt(...)
   local _,angle,id = dok.unpack(
      {...},
      'kinect.tilt',
      [[change the tilt angle of the device]],
      {arg='angle', type='number', help='angle of the tilt', default=0},
      {arg='id', type='number', help='id of the device', default=_kinect.current})
   if id == nil then
      error("No Kinect ON, you need to initDevice first")
   end
   libkinect.tilt(angle,id)
end

function kinect.stop()
   -- stop the thread
   libkinect.stop()
   _kinect.devices = {}
   _kinect.current = nil
end

------------------------------------------------------------
-- Activate the first existing kinect and display RGB and Depth
------------------------------------------------------------
function kinect.testme()
   -- try On/Off
   local dev = kinect.initDevice(0)
   print(dev)
   kinect.stop()
   print(dev)
   dev = kinect.initDevice(0)
   ------------------------------
   -- try grab data
   ------------------------------
   -- RGB
   local rgb,timestampRGB = kinect.getRGB{}
   image.display{image=rgb, legend="time "..timestampRGB}
   -- Depth
   local d,timestampD = kinect.getDepth{}
   image.display{image=d, legend="time "..timestampD}
   -- RGBD
   local rgbd,timestampRGB,timestampD = kinect.getRGBD{}
   title = "time [RGB:"..timestampRGB
   title = title .." Depth:"..timestampD.."]"
   image.display{image=rgbd, legend=title}

   -- tilt
   kinect.tilt(20)
   -- device should turn off automatically with GC
   collectgarbage()
end


return kinect