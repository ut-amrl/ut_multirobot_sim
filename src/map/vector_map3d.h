//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    vector_map.h
\brief   C++ Interfaces: VectorMap3D
\author  Joydeep Biswas, (C) 2010
*/
//========================================================================

#include <vector>
#include <fstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <xmmintrin.h>
#include <smmintrin.h>
#include "geometry.h"
#include "quaternion_helper.h"
//#include "timer.h"
#include "plane_polygon.h"
#include "plane_filtering.h"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/Eigen/Geometry>
#include <unistd.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glx.h>
#include <GL/glu.h>
#include <GL/glext.h>
#include <GL/glut.h>

#include "CImg/CImg.h"
#include "glInfo.h"

#define VECTOR_MAP_3D_SSE_OPTIMIZATIONS

#ifndef VECTOR_MAP_3D_H
#define VECTOR_MAP_3D_H

using namespace std;
using namespace Eigen;

#define GLX_CONTEXT_MAJOR_VERSION_ARB       0x2091
#define GLX_CONTEXT_MINOR_VERSION_ARB       0x2092
typedef GLXContext (*glXCreateContextAttribsARBProc)
    (Display*, GLXFBConfig, GLXContext, Bool, const int*);

/**
A class used to reprsesent a 3D map consisting of planes
**/
class VectorMap3D{
public:

  /// Map Update parameters
  typedef struct{
    /// Maximum angle that the frame polygon's normal can make with the camera
    /// view vector.
    float maxPolygonAngle;
    /// Minimum condition number (ratio of the basis vector eigenvalues),
    /// indicating how "squashed" the polygon is.
    float minConditionNumber;
    /// Minimum number of vertices on the convex hull of the polygon
    int minNumVertices;
    /// Minimum number of points used to construct the plane polygon
    int minNumPoints;
    /// Minimum width of the polygon (length along the major axis of the
    /// polygon)
    float minWidth;
    /// Minimum height of the polygon (length along the minor axis of the
    /// polygon)
    float minHeight;
    /// Max depth of a hole in a polygon to tolerate
    float maxHoleDepth;
  } MapUpdateParams;

  vector<PlanePolygon> polygons;
  string mapsFolder;
  string mapName;
  DepthCam* depthCam;

  bool useWindow;
  // Parameters for colour / index conversion
  double quantizedN;
  double quantizationBins;

  //GLX related vars
  Window win;
  GLXPbuffer pBuffer;
  GLXContext ctx;
  Colormap cmap;
  Display *display;
  static bool ctxErrorOccurred;

  //OpenGL PBO vars
  static const int numPBOs = 2;
  /// Number of channels used in a rendered image, R,G,B,A = 4
  static const int numChannels = 4;
  GLuint imagePBOIds[numPBOs];
  GLuint depthPBOIds[numPBOs];
  GLuint mapDisplayList;
  bool useDMATransfer;

  vector3f prevCameraLoc;
  Quaternionf prevCameraAngle;

  //debug variables
  vector<vector3f> pointsCheckP1,pointsCheckP2;

private:
  /// Initialize OpenGL
  void glInit();

  /// Create a GLX context for off-screen rendering
  void createGLXContext();

  /// Call GL drawing functions to draw the primitives in the map
  void drawMapPrimitives();

  /// Initialize Pixel Buffer Objects to be used for DMA transfers from
  /// Graphics card memory
  void initPBOs();

  /*!
  Helper to check for extension string presence.  Adapted from:
  http://www.opengl.org/resources/features/OGLextensions/
  */
  static bool isExtensionSupported(const char *extList, const char *extension);

  /// Callback to inform the app that a GLX error occured
  static int ctxErrorHandler( Display *dpy, XErrorEvent *ev );

  /// Convert from colour (RGB, 0-255) index to index
  int colourToIndex(double r, double g, double b);

  /// Convert from index to colour index
  void indexToColour(double index,
                     unsigned char& r, unsigned char& g, unsigned char& b);

  /// Generate parameters for colour / index conversion
  void initColourIndex();

  /// SSE implementation of floor()
  double my_floor(double xin);

  void addPolygonsToGroup(vector< PlanePolygon >& group,
                          const vector< int >& overlappingFramePolygons,
                          vector< int >& groupMemberIndices);

  /// Check to see if two groups of polygons overlap.
  bool overlaps(const vector< int >& group1, const vector< int >& group2);

  /// Accepts the vertices of a polygon, merges with the second polygon, and
  /// returns the vertices of the merged polygon.
  vector< vector3f > proposeMerge(vector< vector3f > poly1,
                                  PlanePolygon& poly2);

  /// Returns true if the polygon is consistent with the provided depth image
  bool validMerge(const vector3f& cameraLoc, const Quaternionf& cameraAngle,
                  vector< vector3f > polygon, uint16_t* depth,
                  const VectorMap3D::MapUpdateParams& mapUpdateParams);

public:
  VectorMap3D(const char* name, const char* _mapsFolder,
              DepthCam *_depthCam, bool _useDMATransfer = true,
              bool _useWindow = false);
  ~VectorMap3D();

  void setDepthCamera(DepthCam *_depthCam){depthCam = _depthCam;}

  /// Render the polygons using OpenGL to evaluate ray correspondences
  void renderScene(vector3f cameraLoc, Quaternionf cameraAngle);

  /// Save last rendered scene image to file
  bool saveImage(const char* fname);

  /// Save provided image to file
  bool saveImage(const unsigned char* image, const char* fname);

  /// Load map by name
  bool loadMap(const char* name);

  /// Save map
  bool saveMap();

  /// Add a plane polygon to the map
  void addPolygon(PlanePolygon poly);

  /// Compile the display list for the map
  void compileMap();

  /// Start Render + Readback pipeline : Clears last rendered image, queues next
  /// image to be rendered
  void startRender(vector3f cameraLoc, Quaternionf cameraAngle);

  /// Step to next iteration of Render + Readback pipeline : Retreives last
  /// rendered scene, queues next scene
  void nextRender(vector3f cameraLoc, Quaternionf cameraAngle,
                  unsigned char* image);

  /// Terminates Render + Readback pipeline : Retreives last rendered scene,
  /// queues a dummy scene
  void stopRender(unsigned char* image);

  /// Updates the map by back-projecting the provided depth-image based point
  /// cloud
  void updateMap(vector3f cameraLoc, Quaternionf cameraAngle, vector3f* points);

  /// Updates the map using the provided observed polygons
  void updateMap(vector3f cameraLoc, Quaternionf cameraAngle,
                 const vector< PlanePolygon >& _framePolygons,
                 const VectorMap3D::MapUpdateParams& mapUpdateParams,
                 uint16_t* depth);

  /// Get (read only) access to rendered scene image
  void getRenderedImage(unsigned char* image);

  /// Get (read only) access to rendered scene depth image
  void getDepthImage(float* image);

  /// Start calculation of correspondences using the render pipeline. See
  /// documentation of VectorMap3D::nextCorrespondenceCalc for details.
  void startCorrespondenceCalc(vector3f cameraLoc, Quaternionf cameraAngle);

  /*!
  * \brief Correspondence checking to match specified pixels & points to the
  * \brief rendered scene
  *
  * \param[in] cameraLoc Next Location of camera
  * \param[in] cameraAngle Next Orientation of camera, represented as a
  *                        quaternion
  * \param[in] points 3D points to check for correspondence. points are in the
  *                   reference frame of the camera
  * \param[in] pixels pixel coords of the specified points
  * \param[out] valid Indicates if a corresponding polygon was found for the
  *                   specified points
  * \param[out] indices If valid, the index of the polygon corresponding to the
  *                     specified points
  * \param[out] rays If valid, a ray perpendicular to the corresponding plane,
  *                  from the plane to the specified points
  */
  void nextCorrespondenceCalc(
      vector3f& nextCameraLoc, Quaternionf& nextCameraAngle,
      const vector< vector3f >& points, const vector< vector2i >& pixels,
      vector< bool >& valid, vector< int >& indices,
      vector< vector3f >& rays, vector< vector3f >& transformedPoints);

  /// Finish calculation of correspondences using the render pipeline. See
  /// documentation of VectorMap3D::nextCorrespondenceCalc for details.
  void stopCorrespondenceCalc(
      const vector< vector3f >& points, const vector< vector2i >& pixels,
      vector< bool >& valid, vector< int >& indices,
      vector< vector3f >& rays, vector< vector3f >& transformedPoints);

  /// Get correspondences between rendered image and specified points
  void getCorrespondences(
      vector3f& cameraLoc, Quaternionf& cameraAngle,
      const unsigned char* image, const vector< vector3f >& points,
      const vector< vector2i >& pixels, vector< bool >& valid,
      vector< int >& indices, vector< vector3f >& rays,
      vector< vector3f >& transformedPoints);

  /// Get index of polygon corresponding to specified pixel (0,0=top left
  /// corner, in row major order). Returns true if a valid correspondence was
  /// found.
  bool getCorrespondence(const unsigned char* image, const vector2i& pixel,
                         int& index);

  /// Returns rendered pixel locations corresponding to specified 3D points, as
  /// seen from the camera pose (currentLoc, currentAngle)
  vector<vector3f> renderPoints(const vector3f& currentLoc,
                                const Quaternionf& currentAngle,
                                const vector< vector3f >& points);

  /// Returns the points that were evaluated to check polygons
  void getPointsCheck(vector<vector3f> &p1, vector<vector3f> &p2);
};
#endif //VECTOR_MAP_3D_H
