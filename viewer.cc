#include "./viewer.hpp"

viewer::viewer(){}

viewer::viewer(int w, int h){
  mWidth = w;
  mHeight = h;
  mImage.release();
  mTcb_cv.release();
  mTcb_gl.SetIdentity();
  mbMenuDrawCube = false;
  mbMenuEnd = false;

  // write absolute path
  msModelPath = "/home/vaio/Desktop/araisan/araisan.pmx";
  msMMDDataPath = "/home/vaio/Desktop/araisan";
  msVMDPath = "/home/vaio/Desktop/arau/arau.vmd";
}

viewer::~viewer(){}

bool viewer::SetImage(cv::Mat& image){
  std::unique_lock<std::recursive_mutex> locker = LockerImage();
	cv::cvtColor(image, image, CV_BGR2RGB);
  cv::flip(image, image, 0);
  mImage = image.clone();
  return true;
}

void viewer::SetMenuParams(const bool draw_cube, const bool end) {
  std::unique_lock<std::recursive_mutex> locker = LockerParams();
  mbMenuDrawCube = draw_cube;
  mbMenuEnd = end;
}

void viewer::SetCamPose(const cv::Mat& Tcb) {
  std::unique_lock<std::recursive_mutex> locker = LockerPose();

  mTcb_cv = cv::Mat::eye(4,4,CV_32F);
  cv::Mat m = cv::Mat::eye(4,4,CV_64F);
  cv::Mat cv2gl = (cv::Mat_<double>(4,4) << 1.0,0.0,0.0,0.0,
                                            0.0,-1.0,0.0,0.0,
                                            0.0,0.0,-1.0,0.0,
                                            0.0,0.0,0.0,1.0 );
  Tcb.copyTo(m.rowRange(0,3).colRange(0,4));
  cv::Mat _Tcb;
  _Tcb = cv2gl * m;
  _Tcb.convertTo(mTcb_cv, CV_32F);

  mTcb_gl.m[0] = mTcb_cv.at<float>(0,0);
  mTcb_gl.m[1] = mTcb_cv.at<float>(1,0);
  mTcb_gl.m[2] = mTcb_cv.at<float>(2,0);
  mTcb_gl.m[3] = 0.0;

  mTcb_gl.m[4] = mTcb_cv.at<float>(0,1);
  mTcb_gl.m[5] = mTcb_cv.at<float>(1,1);
  mTcb_gl.m[6] = mTcb_cv.at<float>(2,1);
  mTcb_gl.m[7] = 0.0;

  mTcb_gl.m[8] = mTcb_cv.at<float>(0,2);
  mTcb_gl.m[9] = mTcb_cv.at<float>(1,2);
  mTcb_gl.m[10] = mTcb_cv.at<float>(2,2);
  mTcb_gl.m[11] = 0.0;

  mTcb_gl.m[12] = mTcb_cv.at<float>(0,3);
  mTcb_gl.m[13] = mTcb_cv.at<float>(1,3);
  mTcb_gl.m[14] = mTcb_cv.at<float>(2,3);

  mTcb_gl.m[15] = 1.0;

  return;
}

void viewer::Run(){
/*Load MMD Data---------------*/
  std::string ext = saba::PathUtil::GetExt(msModelPath);
  if(ext == "pmx") {
    auto pmxModel = std::make_unique<saba::PMXModel>();
    if (!pmxModel->Load(msModelPath, msMMDDataPath)) {
      std::cout << "========>>> Load PMXModel Fail" << std::endl;
      return;
    }
    mpModel = std::move(pmxModel);
  }
  else {
    std::cout << "========>>> It is not supported model." << std::endl;
    return;
  }

  auto vmdAnim = std::make_unique<saba::VMDAnimation>();
  if (!vmdAnim->Create(mpModel))
  {
    std::cout << "=======>>> Create VMDAnimation Fail.\n";
    return;
  }

  saba::VMDFile vmdFile;
  if (!saba::ReadVMDFile(&vmdFile, msVMDPath.c_str()))
  {
    std::cout << "=======>>> Read VMD File Fail.\n";
    return;
  }
  if (!vmdAnim->Add(vmdFile))
  {
    std::cout << "======>>> Add VMDAnimation Fail.\n";
    return;
  }
  std::cout << "=======>>> Load MMD Data !\n";

/*---------------Load MMD Data*/
/*Make Pangolin Environment-------------------*/
  pangolin::CreateWindowAndBind("MMD-AR",mWidth,mHeight);

  glEnable(GL_DEPTH_TEST);
  glEnable (GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  
  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(mWidth,mHeight,700,700,mWidth/2,mHeight/2,0.1,100000),
    pangolin::ModelViewLookAt(-1,1,-1,0,0,0,0,-1,0));

  pangolin::View& d_image = pangolin::Display("image")
      .SetBounds(0,1.0,0,1.0,-(double)mWidth/(double)mHeight)
      .SetLock(pangolin::LockLeft, pangolin::LockTop);
  pangolin::GlTexture imageTexture(mWidth,mHeight,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);

  pangolin::View& d_cam = pangolin::Display("cam")
      .SetBounds(0,1.0f,0,1.0f,(double)mWidth/(double)mHeight)
      .SetHandler(new pangolin::Handler3D(s_cam));
/*-------------------Make Pangolin Environment*/
/*Make Saba GL-texture--------------------*/
  std::vector<GLuint> texs;
  std::vector<cv::Mat> imgs;
  texs.resize(mpModel->GetMaterialCount());
  imgs.resize(mpModel->GetMaterialCount());
  // std::cout << "========== Texture List ==========" << std::endl;
  for(size_t i = 0; i < mpModel->GetMaterialCount(); i++) {
    std::string path = mpModel->GetMaterials()[i].m_texture;
    glGenTextures(1, &texs[i]);
    // std::cout << "idx: " << i << " " << mpModel->GetMaterials()[i].m_texture << std::endl;
    imgs[i] = cv::imread(path.c_str());
    //cv::imshow("test", imgs[i]);
    //cv::waitKey(0);
    glBindTexture(GL_TEXTURE_2D, texs[i]);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, 
                  imgs[i].size().width, imgs[i].size().height,
                  0, GL_BGR_EXT, GL_UNSIGNED_BYTE, imgs[i].ptr() );
  }

	glBindTexture(GL_TEXTURE_2D, 0);
  // std::cout << "==================================" << std::endl;
/*--------------------Make Saba texture*/

  {
    // Sync physics animation.
    mpModel->InitializeAnimation();
    vmdAnim->SyncPhysics(0.f * 30.0f);
  }

  while(mImage.empty()) {
    // wait for camera image 
    usleep(10000);
  }

  // main loop
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
  long int count = 0;
  while(!pangolin::ShouldQuit())
  {
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    double current_time = std::chrono::duration_cast<std::chrono::duration<double> >(now - start).count();
#if 1
// https://github.com/sanko-shoko/simplesp/blob/master/sample/xx/mmd/mmd.h
/*Update Animation-----------------------*/
    {
      // Update bone animation.
      mpModel->BeginAnimation();
      vmdAnim->Evaluate((float)current_time/1.0 * 30.0f);
      mpModel->UpdateAnimation();
      mpModel->EndAnimation();

      // Update physics animation.
      mpModel->UpdatePhysics(1.0f / 30.0f);

      // Update vertex.
      mpModel->Update();
    }
    /////////////////////////////////////////////////////////////////////////////////////
    const glm::vec3* positions = mpModel->GetUpdatePositions();
    const size_t vtxCount = mpModel->GetVertexCount();

    const glm::vec3* normals = mpModel->GetUpdateNormals();
    const glm::vec2* uvs = mpModel->GetUpdateUVs();

    std::vector<glm::vec3> vertices;
    for(size_t i = 0; i < vtxCount; i++) {
      vertices.push_back(positions[i]);
    }
/*-----------------------Update Animation*/
#endif
    count++;
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    {
      std::unique_lock<std::recursive_mutex> locker = LockerImage();
      imageTexture.Upload(mImage.data,GL_RGB,GL_UNSIGNED_BYTE);
    }

    d_image.Activate();
    glColor3f(1.0,1.0,1.0);
    imageTexture.RenderToViewport();

    glClear(GL_DEPTH_BUFFER_BIT);
    if(mbMenuDrawCube) {
      std::unique_lock<std::recursive_mutex> locker = LockerPose();
      if(!mTcb_cv.empty()) {
        s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(1,1,1,0,0,0,0,-1,0));
        s_cam.SetModelViewMatrix(mTcb_gl);
      }
      d_cam.Activate(s_cam);
#if 1
/*Draw MMD---------------------*/
      glRotated((double)180, 0.0, 0.0, 1.0);
      glRotated((double)180, 0.0, 1.0, 0.0);
      glTranslated(0.0, -250.0, 0.0);
      glColor3f(1.0,1.0,1.0);
      glEnable(GL_TEXTURE_2D);
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glShadeModel(GL_SMOOTH);

      const saba::MMDSubMesh* subMeshes = mpModel->GetSubMeshes();
      const saba::MMDMaterial* materials = mpModel->GetMaterials();

      std::vector<size_t> indices(mpModel->GetIndexCount());

      if (mpModel->GetIndexElementSize() == 1)
      {
          uint8_t* mmdIndices = (uint8_t*)mpModel->GetIndices();
          for (size_t i = 0; i < indices.size(); i++)
          {
            indices[i] = mmdIndices[i];
          }
      }
      else if (mpModel->GetIndexElementSize() == 2)
      {
          uint16_t* mmdIndices = (uint16_t*)mpModel->GetIndices();
          for (size_t i = 0; i < indices.size(); i++)
          {
            indices[i] = mmdIndices[i];
          }
      }
      else if (mpModel->GetIndexElementSize() == 4)
      {
          uint32_t* mmdIndices = (uint32_t*)mpModel->GetIndices();
          for (size_t i = 0; i < indices.size(); i++)
          {
            indices[i] = mmdIndices[i];
          }
      }
      else
      {
        std::cout << "falied\n";
        return;
      }
      
      for (size_t i = 0; i < mpModel->GetSubMeshCount(); i++) {
        // material
        const saba::MMDMaterial &mat = materials[subMeshes[i].m_materialID];
        const float ambient[4] = { mat.m_ambient.x, mat.m_ambient.y, mat.m_ambient.z, mat.m_alpha };
        const float diffuse[4] = { mat.m_diffuse.x, mat.m_diffuse.y, mat.m_diffuse.z, mat.m_alpha };
        const float specular[4] = { mat.m_specular.x, mat.m_specular.y, mat.m_specular.z, mat.m_alpha };
        glMaterialfv(GL_FRONT, GL_AMBIENT, ambient);
        glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse);
        glMaterialfv(GL_FRONT, GL_SPECULAR, specular);

        // texture
        if(subMeshes[i].m_materialID == 24 
           or subMeshes[i].m_materialID == 22
           or subMeshes[i].m_materialID == 23)
        {
          continue;
        }
        glBindTexture(GL_TEXTURE_2D, texs[subMeshes[i].m_materialID]);
        
        // vertex
        int cnt = subMeshes[i].m_beginIndex;
        for (size_t j = 0; j < subMeshes[i].m_vertexCount; j += 3) {
          glBegin(GL_TRIANGLES);
          for (size_t k = 0; k < 3; k++) {
            const size_t vi = indices[cnt++];
            float m_scale = 18.f; // adjust 
            const glm::vec3 p = m_scale * positions[vi];
            const glm::vec3 n = normals[vi];
            const glm::vec2 uv = uvs[vi];
            
            glTexCoord2f(uv.x, 1-uv.y);
            glNormal3d(n.x, n.y, n.z);
            glVertex3f(p.x, p.y, p.z);
          }
          glEnd();
        }
      }
      glBindTexture(GL_TEXTURE_2D, 0);
      glDisable(GL_TEXTURE_2D);
      glDisable(GL_BLEND);
/*---------------------Draw MMD*/
#else
      glColor3f(1.0,1.0,1.0);
      pangolin::glDrawColouredCube(50);
#endif
    }

    if(mbMenuEnd) {
      std::cout << "=========>>> Finish request is done" << std::endl;
      pangolin::Quit();
    }

    usleep(5000);
    pangolin::FinishFrame();
  }
  std::cout << "=========>>> Destroyed Window" << std::endl;
  return;
}
