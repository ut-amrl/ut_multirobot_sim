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
\file    vector_map3d.cpp
\brief   C++ Implementation: VectorMap3D
\author  Joydeep Biswas, (C) 2010
*/
//========================================================================

#include "vector_map3d.h"

#define DEBUG_FUNCTION_CALLS() \
    printf("%s @ %s:%d\n",__FUNCTION__,__FILE__,__LINE__);


bool VectorMap3D::ctxErrorOccurred = false;

double VectorMap3D::my_floor(double xin)
{
#ifdef VECTOR_MAP_3D_SSE_OPTIMIZATIONS
  __m128d x = _mm_set_sd(xin);
  x = _mm_floor_pd(x);
  double retVal;
  _mm_store_sd(&retVal,x);
  return(retVal);
#else
  return floor(xin);
#endif
}

VectorMap3D::VectorMap3D(const char* name, const char* _mapsFolder,
                         DepthCam *_depthCam, bool _useDMATransfer,
                         bool _useWindow) {
  if (_mapsFolder!=NULL) {
    if (strlen(_mapsFolder)>0)
      mapsFolder = string(_mapsFolder);
    else 
      mapsFolder = string("");
  }else{
    mapsFolder = string("");
  }
  useWindow = _useWindow;
  depthCam = _depthCam;
  useDMATransfer = _useDMATransfer;
  win = 0;
  ctx = 0;
  cmap = 0;
  createGLXContext();
  glInit();
  initPBOs();
  if (name != NULL) {
    if (loadMap(name))
      compileMap();
    else
      printf("Failed to load map %s!\n",name);
  }
}

VectorMap3D::~VectorMap3D()
{
  glXMakeCurrent(display, 0, 0);
  glXDestroyContext(display, ctx);
  if (useWindow) {
    XDestroyWindow(display, win);
    XFreeColormap(display, cmap);
  }
  XCloseDisplay(display);
}

void VectorMap3D::createGLXContext() {
  //Source code adapted from:
  // http://www.opengl.org/wiki/Tutorial:_OpenGL_3.0_Context_Creation_(GLX)
  static const bool debug = false;
  static const bool debugFailures = true;
  display = XOpenDisplay(0);
  if (!display) {
    printf("Failed to open X display\n");
    exit(1);
  }
  // Get a matching FB config
  static int visual_attribs[] = {
    GLX_X_RENDERABLE    , True,
    GLX_DRAWABLE_TYPE   , GLX_PBUFFER_BIT, //GLX_WINDOW_BIT,
    GLX_RENDER_TYPE     , GLX_RGBA_BIT,
    GLX_X_VISUAL_TYPE   , GLX_TRUE_COLOR,
    GLX_RED_SIZE        , 8,
    GLX_GREEN_SIZE      , 8,
    GLX_BLUE_SIZE       , 8,
    GLX_ALPHA_SIZE      , 8,
    GLX_DEPTH_SIZE      , 24,
    GLX_STENCIL_SIZE    , 8,
    GLX_DOUBLEBUFFER    , True,
    //GLX_SAMPLE_BUFFERS  , 1,
    //GLX_SAMPLES         , 4,
    None
  };
  int glx_major, glx_minor;
  // FBConfigs were added in GLX version 1.3.
  if (!glXQueryVersion(display, &glx_major, &glx_minor) ||
      ((glx_major == 1) && (glx_minor < 3)) || (glx_major < 1)) {
    printf("Invalid GLX version");
    exit(1);
  }
  if (debug) printf("Getting matching framebuffer configs\n");
  int fbcount;
  GLXFBConfig *fbc = glXChooseFBConfig(
      display, DefaultScreen(display), visual_attribs, &fbcount);
  if (!fbc) {
    printf("Failed to retrieve a framebuffer config\n");
    exit(1);
  }
  if (debug) printf("Found %d matching FB configs.\n", fbcount);

  // Pick the FB config/visual with the most samples per pixel
  if (debug) printf("Getting XVisualInfos\n");
  int best_fbc = -1, worst_fbc = -1, best_num_samp = -1, worst_num_samp = 999;

  int i;
  for (i = 0; i < fbcount; i++) {
    XVisualInfo *vi = glXGetVisualFromFBConfig(display, fbc[i]);
    if (vi) {
      int samp_buf, samples, pbuffer;
      glXGetFBConfigAttrib(display, fbc[i], GLX_SAMPLE_BUFFERS, &samp_buf);
      glXGetFBConfigAttrib(display, fbc[i], GLX_SAMPLES       , &samples );
      glXGetFBConfigAttrib(display, fbc[i], GLX_DRAWABLE_TYPE   ,&pbuffer );

      if (debug) {
        printf("  Matching fbconfig %d, visual ID 0x%2x: SAMPLE_BUFFERS = %d,"
               "SAMPLES = %d, PBUFFER = %d\n", i, (unsigned int) vi -> visualid,
               samp_buf, samples , ((pbuffer&GLX_PBUFFER_BIT) !=0)?1:0);
      }
      if (best_fbc < 0 || (samp_buf && samples > best_num_samp))
        best_fbc = i, best_num_samp = samples;
      if (worst_fbc < 0 || !samp_buf || samples < worst_num_samp)
        worst_fbc = i, worst_num_samp = samples;
    }
    XFree(vi);
  }
  if (debug) printf("Best FBC:%d\n",best_fbc);
  GLXFBConfig bestFbc = fbc[ best_fbc ];

  // Be sure to free the FBConfig list allocated by glXChooseFBConfig()
  XFree(fbc);

  if (useWindow) {
    // Get a visual
    XVisualInfo *vi = glXGetVisualFromFBConfig(display, bestFbc);
    if (debug) printf("Chosen visual ID = 0x%x\n", (unsigned int) vi->visualid);

    if (debug) printf("Creating colormap\n");
    XSetWindowAttributes swa;
    swa.colormap = cmap = XCreateColormap(
        display, RootWindow(display, vi->screen), vi->visual, AllocNone);
    swa.background_pixmap = None ;
    swa.border_pixel      = 0;
    swa.event_mask        = StructureNotifyMask;
    if (debug) printf("Creating window\n");
    win = XCreateWindow(
        display, RootWindow(display, vi->screen), 0, 0, depthCam->width ,
        depthCam->height, 0, vi->depth, InputOutput, vi->visual,
        CWBorderPixel|CWColormap|CWEventMask, &swa);

    if (!win) {
      printf("Failed to create window.\n");
      exit(1);
    }

    // Done with the visual info data
    XFree(vi);

    XStoreName(display, win, "GL 3.0 Window");
    if (debug) printf("Mapping window\n");
    XMapWindow(display, win);
  }

  // Get the default screen's GLX extension list
  const char *glxExts =
      glXQueryExtensionsString(display, DefaultScreen(display));

  // NOTE: It is not necessary to create or make current to a context before
  // calling glXGetProcAddressARB
  glXCreateContextAttribsARBProc glXCreateContextAttribsARB = 0;
  glXCreateContextAttribsARB =
      (glXCreateContextAttribsARBProc) glXGetProcAddressARB((const GLubyte *)
      "glXCreateContextAttribsARB");
  
  // Install an X error handler so the application won't exit if GL 3.0
  // context allocation fails.
  //
  // Note this error handler is global.  All display connections in all threads
  // of a process use the same error handler, so be sure to guard against other
  // threads issuing X commands while this code is running.
  ctxErrorOccurred = false;
  int (*oldHandler)(Display*, XErrorEvent*) =
      XSetErrorHandler(&ctxErrorHandler);

  // Check for the GLX_ARB_create_context extension string and the function.
  // If either is not present, use GLX 1.3 context creation method.
  if (!isExtensionSupported(glxExts, "GLX_ARB_create_context") ||
      !glXCreateContextAttribsARB) {
    printf("glXCreateContextAttribsARB() not found"
            "... using old-style GLX context\n");
    ctx = glXCreateNewContext(display, bestFbc, GLX_RGBA_TYPE, 0, True);
  }
  else {
    // If it does, try to get a GL 3.0 context!
    int context_attribs[] = {
      GLX_CONTEXT_MAJOR_VERSION_ARB, 3,
      GLX_CONTEXT_MINOR_VERSION_ARB, 0,
      //GLX_CONTEXT_FLAGS_ARB        , GLX_CONTEXT_FORWARD_COMPATIBLE_BIT_ARB,
      None
    };
    if (debug) printf("Creating context\n");
    ctx = glXCreateContextAttribsARB(display, bestFbc, 0, True, context_attribs);
    // Sync to ensure any errors generated are processed.
    XSync(display, False);
    if (!ctxErrorOccurred && ctx) {
      if (debug) printf("Created GL 3.0 context\n");
    } else {
      // Couldn't create GL 3.0 context.  Fall back to old-style 2.x context.
      // When a context version below 3.0 is requested, implementations will
      // return the newest context version compatible with OpenGL versions less
      // than version 3.0.
      // GLX_CONTEXT_MAJOR_VERSION_ARB = 1
      context_attribs[1] = 1;
      // GLX_CONTEXT_MINOR_VERSION_ARB = 0
      context_attribs[3] = 0;
      ctxErrorOccurred = false;
      printf("Failed to create GL 3.0 context ... using old-style GLX context\n");
      ctx = glXCreateContextAttribsARB(display, bestFbc, 0, True, context_attribs);
    }
  }

  // Sync to ensure any errors generated are processed.
  XSync(display, False);

  // Restore the original error handler
  XSetErrorHandler(oldHandler);

  if (ctxErrorOccurred || !ctx) {
    printf("Failed to create an OpenGL context\n");
    exit(1);
  }

  // Verifying that context is a direct context
  if (! glXIsDirect (display, ctx)) {
    if (debug) printf("Indirect GLX rendering context obtained\n");
  } else {
    if (debug) printf("Direct GLX rendering context obtained\n");
  }

  int attrib_list[] = {
    GLX_PBUFFER_WIDTH, depthCam->width ,
    GLX_PBUFFER_HEIGHT, depthCam->height,
    GLX_PRESERVED_CONTENTS, True
  };

  pBuffer = glXCreatePbuffer( display, bestFbc, attrib_list);

  if (useWindow) {
    if (debug) printf("Making context current with window\n");
    glXMakeCurrent(display, win, ctx);
  } else {
    //Use pBuffer
    Bool pBuffSuccess = glXMakeContextCurrent(display, pBuffer, pBuffer, ctx);
    if (debug) {
      printf("Making context current with pBuffer : %s\n",
             (pBuffSuccess==True)?"success":"fail");
    }
  }
}

int VectorMap3D::ctxErrorHandler(Display* dpy, XErrorEvent* ev)
{
  ctxErrorOccurred = true;
  return 0;
}

bool VectorMap3D::isExtensionSupported(
    const char* extList, const char* extension) {
  const char *start;
  const char *where, *terminator;
  /* Extension names should not have spaces. */
  where = strchr(extension, ' ');
  if (where || *extension == '\0')
    return false;
  /* It takes a bit of care to be fool-proof about parsing the
  OpenGL extensions string. Don't be fooled by sub-strings,
  etc. */
  for (start = extList; ;) {
    where = strstr(start, extension);
    if (!where)
      break;
    terminator = where + strlen(extension);
    if (where == start || *(where - 1) == ' ')
      if (*terminator == ' ' || *terminator == '\0')
        return true;
      start = terminator;
  }
  return false;
}

void VectorMap3D::glInit()
{
  glEnable(GL_DEPTH_TEST);
  glDisable(GL_CULL_FACE);
  glShadeModel(GL_FLAT);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 4);      // 4-byte pixel alignment
  glDisable(GL_MULTISAMPLE);
  glEnable(GL_DEPTH_TEST);            // Enables Depth Testing
  glDepthFunc(GL_LEQUAL);             // The Type Of Depth Test To Do
  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_FASTEST);      // Don't care about quality: we need speed!
  
  glClearColor(0.0, 0.0, 0.0, 0.0);
  glClearDepth(1.0f);             // Depth Buffer Setup
  //Viewport and projection
  glViewport(0, 0, depthCam->width, depthCam->height);
  
  //========================Depth Camera Simulator===========================
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  double tanHFovH = tan(depthCam->fovH*0.5); 
  double tanHFovV = tan(depthCam->fovV*0.5);
  glFrustum(-depthCam->minDepth*tanHFovH,depthCam->minDepth*tanHFovH,-depthCam->minDepth*tanHFovV,depthCam->minDepth*tanHFovV,depthCam->minDepth,depthCam->maxDepth);
  //=========================================================================
  
  glMatrixMode(GL_MODELVIEW);
  
  glFinish();
}

void VectorMap3D::initPBOs()
{
  static const bool debug = false;
  static const bool debugFailures = true;
  
  if (debug) printf("Creating pBuffers... ");
  
  glInfo glInfo;
  glInfo.getInfo();
  if (glInfo.isExtensionSupported("GL_ARB_pixel_buffer_object")) {
    if (debug) printf("Video card supports GL_ARB_pixel_buffer_object.");
  }else{
    if (debugFailures) printf("Video card does NOT support GL_ARB_pixel_buffer_object.");
  }
  
  int imageDataSize = depthCam->width*depthCam->height*numChannels;
  unsigned char blankData[imageDataSize];
  memset(blankData, 0, imageDataSize);
  
  glGenBuffersARB(numPBOs, imagePBOIds);
  glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, imagePBOIds[0]);
  glBufferDataARB(GL_PIXEL_PACK_BUFFER_ARB, imageDataSize, blankData, GL_STREAM_READ);
  glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, imagePBOIds[1]);
  glBufferDataARB(GL_PIXEL_PACK_BUFFER_ARB, imageDataSize, blankData, GL_STREAM_READ);
  
  int depthDataSize = depthCam->width*depthCam->height*sizeof(float);
  glGenBuffersARB(numPBOs, depthPBOIds);
  glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, depthPBOIds[0]);
  glBufferDataARB(GL_PIXEL_PACK_BUFFER_ARB, depthDataSize, blankData, GL_STREAM_READ);
  glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, depthPBOIds[1]);
  glBufferDataARB(GL_PIXEL_PACK_BUFFER_ARB, depthDataSize, blankData, GL_STREAM_READ);
  
  glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, 0);
  
  if (debug) printf(" Done!\n");
}

void VectorMap3D::compileMap()
{
  if (glIsList(mapDisplayList)) {
    //Need to delete the list first
    glDeleteLists(mapDisplayList,1);
  }
  // create one display list
  mapDisplayList = glGenLists(1);
  // compile the display list, store the map in it
  glNewList(mapDisplayList, GL_COMPILE);
  drawMapPrimitives();
  glEndList();
  glFinish();
}

void VectorMap3D::drawMapPrimitives()
{
  initColourIndex();
  for(unsigned int i=0; i<polygons.size(); i++) {
    unsigned char r=0,g=0,b=0;
    indexToColour(i,r,g,b);
    glColor3ub(r,g,b);
    
    glBegin(GL_TRIANGLE_FAN);
    glVertex3d(V3COMP(polygons[i].p0));
    for(unsigned int j=0; j<polygons[i].vertices.size(); j++)
      glVertex3d(V3COMP(polygons[i].vertices[j]));
    glVertex3d(V3COMP(polygons[i].vertices[0]));
    glEnd();
  }
}

int VectorMap3D::colourToIndex(double r, double g, double b)
{
  double scale = quantizedN/256.0;
  double ind = my_floor(b*scale) + quantizedN*(my_floor(g*scale) + quantizedN*my_floor(r*scale));
  return my_floor(ind-1.0);
}

void VectorMap3D::indexToColour(double index, unsigned char& r, unsigned char& g, unsigned char& b)
{
  double scale = 255.0/quantizedN;
  double eps = 0.5*scale + 0.5;
  index = my_floor(index)+1.0;
  b = my_floor(fmod(index,quantizedN)*scale+eps);
  index = my_floor(index/quantizedN);
  g = my_floor(fmod(index,quantizedN)*scale+eps);
  index = my_floor(index/quantizedN);
  r = my_floor(fmod(index,quantizedN)*scale+eps);
}

void VectorMap3D::initColourIndex()
{
  quantizedN = ceil(pow(2.0,ceil(log(double(polygons.size()+1))/(3.0*log(2.0)))));
}

void VectorMap3D::addPolygon(PlanePolygon poly)
{
  polygons.push_back(poly);
}

bool VectorMap3D::saveMap()
{
  char fileName[4096];
  FILE * pFile;
  snprintf(fileName,4095,"%s/%s/%s_3d.txt",mapsFolder.c_str(),mapName.c_str(),mapName.c_str());
  pFile = fopen(fileName,"w");
  if (pFile == NULL) {
    return false;
  }
  
  for(unsigned int i=0; i<polygons.size(); i++) {
    fprintf(pFile,"%d, ",int(polygons[i].vertices.size()));
    for(unsigned int j=0; j<polygons[i].vertices.size(); j++) {
      fprintf(pFile,"%.4f,%.4f,%.4f, ",V3COMP(polygons[i].vertices[j]));
    }
    fprintf(pFile,"\n");
  }
  fclose(pFile);
  return true;
}


bool VectorMap3D::loadMap(const char* name)
{
  static const bool debug = false;
  
  char fileName[4096];
  FILE * pFile;
  mapName = string(name);
  snprintf(fileName,4095,"%s/%s/%s_3d.txt",mapsFolder.c_str(),name,name);
  pFile = fopen(fileName,"r");
  if (pFile == NULL) {
    return false;
  }
  bool error = false;
  
  while(!error) {
    int numVertices = 0;
    vector<vector3f> points;
    error = (fscanf(pFile,"%d, ",&numVertices)!=1);
    if (debug && !error) {
      if (debug) printf("%d:\n",int(polygons.size()));
    }
    for(int i=0; i<numVertices && !error; i++) {
      float x=0,y=0,z=0;
      vector3f v;
      error = (fscanf(pFile,"%f,%f,%f, ",&x,&y,&z)!=3);
      if (!error) {
        v.set(x,y,z);
        points.push_back(v);
        if (debug) printf("%f,%f,%f\n",x,y,z);
      }
    }
    if (!error) {
      PlanePolygon poly(points);
      if (poly.validPolygon)
        polygons.push_back(poly);
      else if (debug)
        printf("Invalid map polygon!\n");
    }
    if (debug) printf("\n");
  }
  fclose(pFile);
  
  if (debug) printf("Done loading map\n\n");
  return true;
}

void VectorMap3D::renderScene(vector3f cameraLoc, Quaternionf cameraAngle)
{
  vector3f lookAt(1.0, 0.0, 0.0);
  vector3f upVector(0.0, 0.0, 1.0);
  lookAt = GVector::quat_rotate(cameraAngle,lookAt);
  upVector = GVector::quat_rotate(cameraAngle,upVector);
  
  glDrawBuffer(GL_BACK);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();
  
  //glTranslatef(0.0, 0.0, -10.0);
  gluLookAt(V3COMP(cameraLoc),
            V3COMP(cameraLoc + lookAt),
            V3COMP(upVector));

  // draw the display list
  //glCallList(mapDisplayList);
  drawMapPrimitives();
}

void VectorMap3D::getRenderedImage(unsigned char* image)
{
  static const bool debug = true;
  
  int imageDataSize = depthCam->width*depthCam->height*numChannels;
  static int index = 0;
  int nextIndex = 0;                  // pbo index used for next frame
  // increment current index first then get the next index
  // "index" is used to read pixels from a framebuffer to a PBO
  // "nextIndex" is used to process pixels in the other PBO
  index = (index + 1) % 2;
  nextIndex = (index + 1) % 2;
  
  glFinish();
  if (useWindow)
    glXSwapBuffers (display, win);
  else
    glXSwapBuffers (display, pBuffer);
  glReadBuffer(GL_FRONT);
  if (useDMATransfer) {
    // PBO glReadPixels
    glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, imagePBOIds[index]);
    glReadPixels(0, 0, depthCam->width , depthCam->height, GL_BGRA, GL_UNSIGNED_BYTE, 0);
    glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, imagePBOIds[nextIndex]);
    void* mem = glMapBufferARB(GL_PIXEL_PACK_BUFFER_ARB, GL_READ_ONLY_ARB); //blocks till data is available
    if (mem) {
      memcpy(image, mem, imageDataSize);
      glUnmapBufferARB(GL_PIXEL_PACK_BUFFER_ARB);
    }else{
      if (debug) printf("glMapBufferARB failed!\n");
    }
    glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, 0);
  }else{
    glReadPixels(0, 0, depthCam->width , depthCam->height, GL_BGRA, GL_UNSIGNED_BYTE, image);
  }
}

void VectorMap3D::getDepthImage(float* image)
{
  static const bool debug = true;
  
  int depthDataSize = depthCam->width*depthCam->height*sizeof(float);
  static int index = 0;
  int nextIndex = 0;                  // pbo index used for next frame
  // increment current index first then get the next index
  // "index" is used to read pixels from a framebuffer to a PBO
  // "nextIndex" is used to process pixels in the other PBO
  index = (index + 1) % 2;
  nextIndex = (index + 1) % 2;
  
  glFinish();
  if (useWindow)
    glXSwapBuffers (display, win);
  else
    glXSwapBuffers (display, pBuffer);
  glReadBuffer(GL_FRONT);
  if (useDMATransfer) {
    // PBO glReadPixels
    glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, depthPBOIds[index]);
    glReadPixels(0, 0, depthCam->width , depthCam->height, GL_DEPTH_COMPONENT, GL_FLOAT, 0);
    glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, depthPBOIds[nextIndex]);
    void* mem = glMapBufferARB(GL_PIXEL_PACK_BUFFER_ARB, GL_READ_ONLY_ARB); //blocks till data is available
    if (mem) {
      memcpy(image, mem, depthDataSize);
      glUnmapBufferARB(GL_PIXEL_PACK_BUFFER_ARB);
    }else{
      if (debug) printf("glMapBufferARB failed!\n");
    }
    glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, 0);
  }else{
    glReadPixels(0, 0, depthCam->width , depthCam->height, GL_DEPTH_COMPONENT, GL_FLOAT, image);
  }
}

void VectorMap3D::startRender(vector3f cameraLoc, Quaternionf cameraAngle)
{
  // Start rendering scene to initiate the pipeline
  renderScene(cameraLoc, cameraAngle);
  // Extract last rendered dummy scene
  int imageDataSize = depthCam->width*depthCam->height*numChannels;
  unsigned char dummyImage[imageDataSize];
  getRenderedImage(dummyImage);
}

void VectorMap3D::nextRender(vector3f cameraLoc, Quaternionf cameraAngle, unsigned char* image)
{
  // Render the next scene
  renderScene(cameraLoc, cameraAngle);
  // Extract the last scene
  getRenderedImage(image);
}

void VectorMap3D::stopRender(unsigned char* image)
{
  // Dummy scene render to complete the pipeline: render a scene with the camera at the origin
  renderScene(prevCameraLoc,prevCameraAngle);
  // Extract last rendered image
  getRenderedImage(image);
}

void VectorMap3D::startCorrespondenceCalc(vector3f cameraLoc, Quaternionf cameraAngle)
{
  static const bool debug = false;
  startRender(cameraLoc, cameraAngle);
  prevCameraLoc = cameraLoc;
  prevCameraAngle = cameraAngle;
  if (debug) printf(" Render start at %.3f,%.3f,%.3f @ %f,%f,%f,%f\n",V3COMP(cameraLoc), cameraAngle.w(), cameraAngle.x(), cameraAngle.y(), cameraAngle.z());
}

void VectorMap3D::nextCorrespondenceCalc(vector3f& nextCameraLoc, Quaternionf& nextCameraAngle, const std::vector< vector3f >& points, const std::vector< vector2i >& pixels, std::vector< bool >& valid, std::vector< int >& indices, vector< vector3f >& rays, vector< vector3f >& transformedPoints)
{
  int imageDataSize = depthCam->width*depthCam->height*numChannels;
  unsigned char image[imageDataSize];
  nextRender(nextCameraLoc, nextCameraAngle, image);
  getCorrespondences(prevCameraLoc, prevCameraAngle, image, points, pixels, valid, indices, rays, transformedPoints);
  prevCameraLoc = nextCameraLoc;
  prevCameraAngle = nextCameraAngle;
}

void VectorMap3D::stopCorrespondenceCalc(const std::vector< vector3f >& points, const std::vector< vector2i >& pixels, std::vector< bool >& valid, std::vector< int >& indices, vector< vector3f >& rays, vector< vector3f >& transformedPoints)
{
  int imageDataSize = depthCam->width*depthCam->height*numChannels;
  unsigned char image[imageDataSize];
  stopRender(image);
  getCorrespondences(prevCameraLoc, prevCameraAngle, image, points, pixels, valid, indices, rays, transformedPoints);
}

void VectorMap3D::getCorrespondences(vector3f& cameraLoc, Quaternionf& cameraAngle, const unsigned char* image, const std::vector< vector3f >& points, const std::vector< vector2i >& pixels, std::vector< bool >& valid, std::vector< int >& indices, vector< vector3f >& rays, vector< vector3f >& transformedPoints)
{
  int n = points.size();
  valid.resize(n);
  indices.resize(n);
  rays.resize(n);
  
  GVector::matrix3d<float> M;
  GVector::transformMatrix<float>(cameraAngle, cameraLoc, M);
  for(int i=0; i<n; i++) {
    valid[i] = getCorrespondence(image, pixels[i], indices[i]);
    if (valid[i]) {
      transformedPoints[i] = points[i].transform(M);
      rays[i] = polygons[indices[i]].rayFromPlane(transformedPoints[i]);
      
    }else{
      rays[i].zero();
    }
  }
  
}

inline bool VectorMap3D::getCorrespondence(const unsigned char* image, const vector2i& pixel, int& index)
{
  int loc = 4*((depthCam->height-1-pixel.y)*depthCam->width+pixel.x);
  int b = image[loc], g = image[loc+1], r = image[loc+2];
  index = colourToIndex(r,g,b);
  return (index>=0 && index<int(polygons.size()) && (r+g+b>0));
}

bool VectorMap3D::saveImage(const unsigned char* image, const char* fname)
{
  unsigned char image2[depthCam->width * depthCam->height*3];
  memset(image2,0,depthCam->width * depthCam->height*3);
  for(unsigned int y=0; y<depthCam->height; y++) {
    for(unsigned int x=0; x<depthCam->width; x++) {
      int glPixelIndex = 4*(y*depthCam->width+x);
      image2[depthCam->width *(depthCam->height-y-1)+x + 0*depthCam->width * depthCam->height] = image[glPixelIndex+2];
      image2[depthCam->width *(depthCam->height-y-1)+x + 1*depthCam->width * depthCam->height] = image[glPixelIndex+1];
      image2[depthCam->width *(depthCam->height-y-1)+x + 2*depthCam->width * depthCam->height] = image[glPixelIndex+0];
    }
  }
  cimg_library::CImg<unsigned char> img(image2,depthCam->width ,depthCam->height,1,3,false);
  img.save_bmp(fname);
  if (true) {
    FILE* fid = fopen("image.dat","w");
    fwrite(image2, sizeof(unsigned char),depthCam->width * depthCam->height*3, fid);
    fclose(fid);
  }
  return true;
}

bool VectorMap3D::saveImage(const char* fname)
{
  int imageDataSize = depthCam->width*depthCam->height*numChannels;
  unsigned char image[imageDataSize];
  //getRenderedImage(image);
  getDepthImage((float*)image);
  if (image==NULL)
    return false;
  return saveImage(image,fname);
}

void VectorMap3D::updateMap(vector3f cameraLoc, Quaternionf cameraAngle, vector3f* points)
{
  static const float maxDepthDiff = 0.05;
  vector<vector3f> polygonPoints(4);
  // indices in counter-clockwise order for the current quad
  unsigned int ind1, ind2, ind3, ind4;
  polygons.clear();
  for(unsigned int i=0; i<depthCam->height-1; i++) {
    for(unsigned int j=0; j<depthCam->width-1; j++) {
      //Check if the depth change is within bounds
      if (fabs(points[ind1].x - points[ind2].x)<maxDepthDiff && fabs(points[ind1].x - points[ind3].x)<maxDepthDiff && fabs(points[ind1].x - points[ind4].x)<maxDepthDiff) {
        polygonPoints[0] = points[ind1];
        polygonPoints[1] = points[ind2];
        polygonPoints[2] = points[ind3];
        polygonPoints[3] = points[ind3];
        
        polygonPoints[0].set(1.0, -1.0, -1.0);
        polygonPoints[1].set(1.0, 1.0, -1.0);
        polygonPoints[2].set(1.0, 1.0, 1.0);
        polygonPoints[3].set(1.0, -1.0, 1.0);
        PlanePolygon poly(polygonPoints);
        if (poly.validPolygon)
          polygons.push_back(poly);
      }
    }
  }
}

vector< vector3f> VectorMap3D::renderPoints(const vector3f &currentLoc, const Quaternionf &currentAngle, const vector< vector3f >& points)
{
  int n = points.size();
  vector<vector3f> renderedPoints(n);
  
  GVector::matrix3d<float> M, M1, M2;
  GVector::transformMatrix<float>(Quaternionf::Identity(),-currentLoc,M1);
  GVector::transformMatrix<float>(currentAngle.inverse(),vector3f(0.0,0.0,0.0),M2);
  M = M2*M1;
  
  vector3f p;
  float tanHFovH = 1.0/tan(depthCam->fovH*0.5); 
  float tanHFovV = 1.0/tan(depthCam->fovV*0.5);
  float h = float(depthCam->height)*0.5;
  float w = float(depthCam->width)*0.5;
  for(int i=0; i<n; i++) {
    p = points[i].transform(M);
    renderedPoints[i].set((-p.y/p.x*tanHFovH+1.0)*w , (-p.z/p.x*tanHFovV+1.0)*h,p.x);
  }
  
  return renderedPoints;
}

vector< vector3f > VectorMap3D::proposeMerge(vector< vector3f > poly1, PlanePolygon& poly2)
{
  static GrahamsScan grahams;
  vector<vector2f> points = poly2.vertices2D;
  for(unsigned int i=0; i<poly1.size(); i++) {
    points.push_back(poly2.projectOnto(poly1[i]));
  }
  points = grahams.run(points);
  
  vector<vector3f> mergedPoly(points.size());
  for(unsigned int i=0; i<points.size(); i++) {
    mergedPoly[i] = poly2.b1*points[i].x + poly2.b2*points[i].y + poly2.p0;
  }
  return mergedPoly;
}

bool VectorMap3D::validMerge(const vector3f &cameraLoc, const Quaternionf &cameraAngle, vector< vector3f > polygon, uint16_t* depth, const MapUpdateParams &mapUpdateParams)
{
  #define ENABLE_DEBUG_POINTS_CHECK
  
  bool valid = true;
  int n = polygon.size();
  vector<vector3f> points(n);
  for(int i=0; i<n; i++) {
    points[i] = 0.5*(polygon[i]+polygon[(i+1)%n]);
  }
  vector<vector3f> pixelLocs = renderPoints(cameraLoc,cameraAngle,points);
  
  for(int i=0; valid && i<n; i++) {
    if (int(pixelLocs[i].y)>479 || int(pixelLocs[i].y)<0 || int(pixelLocs[i].x)>639 || int(pixelLocs[i].x)<0)
      continue;
    int ind  = int(pixelLocs[i].y)*640+int(pixelLocs[i].x);
    float d = depthCam->rawDepthToMetricDepth(&(depth[ind]));
    
    valid  = (fabs(d-pixelLocs[i].z)<mapUpdateParams.maxHoleDepth) || !depthCam->isValidDepth(pixelLocs[i].y, pixelLocs[i].x, depth);
  }

  #ifdef ENABLE_DEBUG_POINTS_CHECK
  GVector::matrix3d<float> transform;
  GVector::transformMatrix<float>(cameraAngle,cameraLoc, transform);
  for(int i=0; !valid && i<n; i++) {
    if (int(pixelLocs[i].y)>479 || int(pixelLocs[i].y)<0 || int(pixelLocs[i].x)>639 || int(pixelLocs[i].x)<0)
      continue;
    pointsCheckP1.push_back(points[i]);
    int ind  = int(pixelLocs[i].y)*640+int(pixelLocs[i].x);
    pointsCheckP2.push_back(depthCam->depthPixelTo3D(ind,depth).transform(transform));
    //printf("%f,%f,%f : %f,%f,%f\n",V3COMP(points[i]),V3COMP(depthCam->depthPixelTo3D(ind,depth)));
  }
  #endif

  return valid;
}

void VectorMap3D::getPointsCheck(vector< vector3f >& p1, vector< vector3f >& p2)
{
  p1 = pointsCheckP1;
  p2 = pointsCheckP2;
  pointsCheckP1.clear();
  pointsCheckP2.clear();
}

void VectorMap3D::updateMap(vector3f cameraLoc, Quaternionf cameraAngle, const vector< PlanePolygon >& _framePolygons, const MapUpdateParams &mapUpdateParams, uint16_t* depth)
{
  static bool debug = false;
  const float MinCosineError = cos(RAD(20.0));
  const float MaxOffsetError = 0.04;
  int polygonIndex;
  bool valid;
  float cosineError, offsetError;
  vector< PlanePolygon > framePolygons = _framePolygons;
  
  vector< vector<int> > overlappingFramePolygons(framePolygons.size());
  vector<bool> framePolygonOccluded(framePolygons.size());
  vector<int> overlappingMapPolygons(polygons.size());
  vector<int> curOverlappingFramePolygons;
  int imageDataSize = depthCam->width*depthCam->height*numChannels;
  unsigned char image[imageDataSize];
  //Perform a scene render using current location
  startRender(cameraLoc,cameraAngle);
  stopRender(image);  
  
  //Determine polygons (observed vs. map) overlap
  
  GVector::matrix3d<float> transformMatrix;
  GVector::transformMatrix<float>(cameraAngle,cameraLoc,transformMatrix);
  vector3f viewVector(1.0,0.0,0.0, 0.0);
  viewVector = viewVector.transform(transformMatrix);
  
  if (debug) {
    printf("Map update: %.3f,%.3f,%.3f @ %.6f,%.6f,%.6f,%.6f (%.3f\u00b0)\n",
           V3COMP(cameraLoc), cameraAngle.w(), cameraAngle.x(), cameraAngle.y(), cameraAngle.z(),DEG(cameraAngle.angularDistance(Quaternionf(1.0f,0.0f,0.0f,0.0f))));
  }
  
  vector<vector3f> proposedMergedPolygon, mergedPolygon;
  for(unsigned int i=0; i<framePolygons.size(); i++) {
    curOverlappingFramePolygons.clear();
    framePolygonOccluded[i] = false;
    framePolygons[i].transform(transformMatrix);
    mergedPolygon = framePolygons[i].vertices;
    
    for(unsigned int j=0; j<framePolygons[i].pixelLocs.size(); j++) {
      valid = getCorrespondence(image, framePolygons[i].pixelLocs[j], polygonIndex);
      if (valid) {
        cosineError = fabs(framePolygons[i].normal.dot(polygons[polygonIndex].normal));
        offsetError = max(fabs(framePolygons[i].normal.dot(polygons[polygonIndex].p0 - framePolygons[i].p0)), fabs(polygons[polygonIndex].normal.dot(polygons[polygonIndex].p0 - framePolygons[i].p0)));
        
        if (viewVector.dot(framePolygons[i].p0-polygons[polygonIndex].p0) >0.0) {
          framePolygonOccluded[i] = true;
        }
        
        if (cosineError>MinCosineError && offsetError<MaxOffsetError) {
          
          proposedMergedPolygon = proposeMerge(mergedPolygon,polygons[polygonIndex]);
          //validMerge(cameraLoc, cameraAngle, framePolygons[i].vertices, depth, mapUpdateParams);
          //validMerge(cameraLoc, cameraAngle, mergedPolygon, depth, mapUpdateParams);
          //validMerge(cameraLoc, cameraAngle, proposedMergedPolygon , depth, mapUpdateParams);
          
          if (!validMerge(cameraLoc, cameraAngle, proposedMergedPolygon, depth, mapUpdateParams))
            continue;
          
          mergedPolygon = proposedMergedPolygon;
          if (curOverlappingFramePolygons.size()==0) {
            curOverlappingFramePolygons.push_back(polygonIndex);
          }else if (*std::find(curOverlappingFramePolygons.begin(),curOverlappingFramePolygons.end(),polygonIndex) != polygonIndex) {
            curOverlappingFramePolygons.push_back(polygonIndex);
          }
        }
      }
    }
    if (debug) {
      printf("frame poly %d matches: ",i);
      for(unsigned int j=0; j<curOverlappingFramePolygons.size(); j++)
        printf("%d ",curOverlappingFramePolygons[j]);
      printf("\n");
    }
    overlappingFramePolygons[i] = curOverlappingFramePolygons;
  }
  
  // Determine which polygons of the map need to be merged
  vector< vector<int> > polygonMergingGroupIndices;
  vector<vector<PlanePolygon> > polygonMergingGroups;
  vector<PlanePolygon> currentGroup;
  vector<int> currentGroupMemberIndices;
  int numGroups = 0;
  
  for(unsigned int i=0; i<overlappingMapPolygons.size(); i++)
    overlappingMapPolygons[i] = -1;
  
  
  for(unsigned int i=0; i<framePolygons.size(); i++) {
    
    if (overlappingFramePolygons[i].size()==0) {
      if (fabs(framePolygons[i].normal.dot(viewVector))<cos(mapUpdateParams.maxPolygonAngle) ||
        fabs(framePolygons[i].conditionNumber)<mapUpdateParams.minConditionNumber || 
        framePolygons[i].vertices.size()<mapUpdateParams.minNumVertices || 
        framePolygons[i].width<mapUpdateParams.minWidth || 
        framePolygons[i].height<mapUpdateParams.minHeight || 
        framePolygons[i].numPoints<mapUpdateParams.minNumPoints ||
        framePolygonOccluded[i]) { //According to the map, this frame polygon should be occluded. Either ignore, or carve a hole in the map.
       
        continue; //ignore the frame polygon
      }
      // this frame polygon did not match any polygon on the map: add it!
      polygons.push_back(framePolygons[i]);
      continue;
    }
    currentGroupMemberIndices.clear();
    currentGroup.clear();
    addPolygonsToGroup(currentGroup,overlappingFramePolygons[i], currentGroupMemberIndices);
    currentGroup.push_back(framePolygons[i]);
    for(unsigned int j=i+1; j<overlappingFramePolygons.size(); j++) {
      if (overlaps(overlappingFramePolygons[i],overlappingFramePolygons[j])) {
        addPolygonsToGroup(currentGroup,overlappingFramePolygons[j], currentGroupMemberIndices);
        currentGroup.push_back(framePolygons[j]);
        framePolygons.erase(framePolygons.begin()+j);
        overlappingFramePolygons.erase(overlappingFramePolygons.begin()+j);
        j--;
      }
    }
    polygonMergingGroupIndices.push_back(currentGroupMemberIndices);
    polygonMergingGroups.push_back(currentGroup);
  }
  
  if (debug) {
    printf("\n%d groups\n",int(polygonMergingGroups.size()));
    for(unsigned int i=0; i<polygonMergingGroupIndices.size(); i++) {
      printf("\nGroup %d: ",i);
      for(unsigned int j=0; j<polygonMergingGroupIndices[i].size(); j++) {
        printf("%d ",polygonMergingGroupIndices[i][j]);
      }
      printf("\n");
    }
  }
  
  //Merge polygons: Update existing matched polygon parameters and observation counter
  vector<int> duplicatePolygonIndices;
  for(unsigned int i=0; i<polygonMergingGroups.size(); i++) {
    int rootPolygonIndex = polygonMergingGroupIndices[i][0];
    polygonMergingGroups[i].erase(polygonMergingGroups[i].begin());
    polygons[rootPolygonIndex].merge(polygonMergingGroups[i]);
    duplicatePolygonIndices.insert(duplicatePolygonIndices.begin(),polygonMergingGroupIndices[i].begin()+1,polygonMergingGroupIndices[i].end());
  }
  std::sort(duplicatePolygonIndices.begin(), duplicatePolygonIndices.end());
  if (debug) {
    printf("duplicate polygons: ");
    for(unsigned int i=0; i<duplicatePolygonIndices.size(); i++) {
      printf("%d ",duplicatePolygonIndices[i]);
    }
    printf("\n");
  }
  
  //Remove duplicate polygons (since they've been merged with other polygons)
  int lastIndex = -1;
  for(int i=duplicatePolygonIndices.size()-1; i>=0; i--) {
    if (duplicatePolygonIndices[i]==lastIndex)
      continue;
    lastIndex = duplicatePolygonIndices[i];
    if (debug) printf("erasing map poly %d of %d\n",duplicatePolygonIndices[i], int(polygons.size()));
    polygons.erase(polygons.begin()+duplicatePolygonIndices[i]);
  }
    
  //Remove polygons with sufficiently low observation count
  // TODO
}

bool VectorMap3D::overlaps(const std::vector< int >& group1, const std::vector< int >& group2)
{
  for(unsigned int i=0; i<group1.size(); i++) {
    for(unsigned int j=0; j<group2.size(); j++)
      if (group1[i]==group2[j])
        return true;
  }
  return false;
}


void VectorMap3D::addPolygonsToGroup(vector< PlanePolygon > &group, const vector< int >& overlappingFramePolygons, vector< int >& groupMemberIndices)
{
  for(unsigned int i=0; i<overlappingFramePolygons.size(); i++) {
    bool found = false;
    for(unsigned int j=0; j<groupMemberIndices.size() && !found; j++) {
      found = groupMemberIndices[j] == overlappingFramePolygons[i];
    }
    if (found)
      continue;
    groupMemberIndices.push_back(overlappingFramePolygons[i]);
    group.push_back(polygons[overlappingFramePolygons[i]]);
  }
}
