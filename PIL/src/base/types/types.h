/******************************************************************************

  Robot Toolkit ++ (RTK++)

  Copyright (c) 2007-2013 Shuhui Bu <bushuhui@nwpu.edu.cn>
  http://www.adv-ci.com

  ----------------------------------------------------------------------------

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.

*******************************************************************************/

#ifndef __ZY_TYPE_H__
#define __ZY_TYPE_H__

#include <stdio.h>
#include <stdint.h>
#include <string>
#include <sstream>
#include <math.h>

//#define HAS_TOON
#ifdef HAS_TOON
#include <TooN/TooN.h>
#endif


#include <GSLAM/core/Array.h>
#include <GSLAM/core/Point.h>


namespace pi {

/******************************************************************************
 * basic types
 *****************************************************************************/

typedef uint8_t         ru8;
typedef uint16_t        ru16;
typedef uint32_t        ru32;
typedef uint64_t        ru64;

typedef int8_t          ri8;
typedef int16_t         ri16;
typedef int32_t         ri32;
typedef int64_t         ri64;

typedef float           rf32;
typedef double          rf64;

typedef unsigned char   byte;



/******************************************************************************
 * graphics types
 *****************************************************************************/
// color
struct RTK_Color {
    ru8   r, g, b, a;

    RTK_Color() {
        r = 0x00;
        g = 0x00;
        b = 0x00;
        a = 0xFF;
    }

    void set(ru8 _r, ru8 _g, ru8 _b, ru8 _a=0xff) {
        r = _r;
        g = _g;
        b = _b;
        a = _a;
    }
};

} // end namespace pi

#endif // __RTK_TYPE_H__

