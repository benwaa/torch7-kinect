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
_kinect.IDs = {}
_kinect.current = nil -- current device in use

function kinect.initDevice(...)
   local _,id = dok.unpack(
      {...},
      'kinect.device',
      [[return the current device from frame grabbing]],
      {arg='id', type='number', help='id of the device', default=0})
   if _kinect.devices[id] == nil then
      _kinect.devices[id] = libkinect.newdevice(id)
      -- add the id to the list of IDs (to remember who is initiated)
      table.insert(_kinect.IDs, id)
      -- set the led to show it's working
      kinect.led{color='green',id=id}
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
      [[return the current RGB frame]],
      {arg='id', type='number', help='id of the device', default=_kinect.current})
   if id == nil then
      error("No Kinect ON, you need to initDevice first")
   end
   local rgb = torch.Tensor(3,480,640)
   rgb.libkinect.grabRGB(rgb,id)
   return rgb
end

function kinect.led(...)
   -- list of colors
   local colors = {off=0,green=1,red=2,orange=3,
                   green_wink=4,
                   orange_wink_red=6}
   local choices = ''
   for k,_ in pairs(colors) do choices = choices .. ',"' .. k .. '"' end
   choices ='['..string.gsub(choices,"^,","") .. ']'
   local _,color,id = dok.unpack(
      {...},
      'kinect.led',
      [[change the LED color of the device]],
      {arg='color', type='number',
       help='new color for the LED among '..choices, default='orange_wink_red'},
      {arg='id', type='number', help='id of the device',
       default=_kinect.current})
   if id == nil then
      error("No Kinect ON, you need to initDevice first")
   end
   local value = colors[color]
   if value == nil then
      error("Color unknow, choose among "..choices)
   end
   libkinect.led(value,_kinect.devices[id])
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
   libkinect.stop(_kinect.IDs,_kinect.devices)
   -- turn off all devices' flag
   for _,id in ipairs(_kinect.IDs) do
      libkinect.turnOff(_kinect.devices[id])
   end
   _kinect.devices = {}
   _kinect.current = nil
end

------------------------------------------------------------
-- Activate the first existing kinect and display RGB and Depth
------------------------------------------------------------
function kinect.testme()
   kinect.initDevice(0)
   image.display(kinect.getRGB{})
   kinect.tilt(20)
end


return kinect