#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

//#define STB_IMAGE_IMPLEMENTATION
//#include <stb_image.h>

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <filesystem>

#include "mesh.h"

class FBXMesh {
private:
  std::vector<Mesh> meshes;
  float scale;

  unsigned int loadTexture(const std::string& p);
  Mesh loadMesh(const aiMesh* m, const aiScene* scene, std::string base_path);
  
public:
  FBXMesh(std::string filename, float scale=1.f);
  void draw();
};
