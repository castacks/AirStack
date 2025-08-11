#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>

#include "mesh.h"

class ColladaMesh {
private:
  std::vector<Mesh> meshes;
  
  Mesh loadMesh(const aiMesh* mesh);
  
public:
  ColladaMesh(std::string filename);
  void draw();
};
