#include "mesh_collada.h"

ColladaMesh::ColladaMesh(std::string filename){
  Assimp::Importer importer;
  const aiScene* scene = importer.ReadFile(filename,
					   aiProcess_Triangulate);

  if (!scene || !scene->HasMeshes()) {
    std::cerr << "Model load failed: " << importer.GetErrorString() << "\n";
    return;
  }

  // load all meshes
  for (unsigned int i = 0; i < scene->mNumMeshes; ++i) {
    aiMesh* mesh = scene->mMeshes[i];
    std::cout << "Loaded mesh " << i << " with " << mesh->mNumVertices << " vertices" << std::endl;
    meshes.push_back(loadMesh(mesh));
  }
}

void ColladaMesh::draw(){
  for(Mesh& mesh : meshes){
    glBindVertexArray(mesh.VAO);
    glDrawElements(GL_TRIANGLES, mesh.indexCount, GL_UNSIGNED_INT, 0);
  }
}

void ColladaMesh::draw_except(int index){
  for(int i = 0; i < meshes.size(); i++){
    if(i == index)
      continue;
    Mesh& mesh = meshes[i];
    glBindVertexArray(mesh.VAO);
    glDrawElements(GL_TRIANGLES, mesh.indexCount, GL_UNSIGNED_INT, 0);
  }
}

void ColladaMesh::draw_only(int index){
  Mesh& mesh = meshes[index];
  glBindVertexArray(mesh.VAO);
  glDrawElements(GL_TRIANGLES, mesh.indexCount, GL_UNSIGNED_INT, 0);
}

Mesh ColladaMesh::loadMesh(const aiMesh* mesh) {
    std::vector<float> vertices;
    std::vector<unsigned int> indices;

    for (unsigned i = 0; i < mesh->mNumVertices; i++) {
        vertices.push_back(mesh->mVertices[i].x);
        vertices.push_back(mesh->mVertices[i].y);
        vertices.push_back(mesh->mVertices[i].z);
	
        vertices.push_back(mesh->mNormals[i].x);
        vertices.push_back(mesh->mNormals[i].y);
        vertices.push_back(mesh->mNormals[i].z);
    }

    for (unsigned i = 0; i < mesh->mNumFaces; i++) {
        for (unsigned j = 0; j < mesh->mFaces[i].mNumIndices; j++)
            indices.push_back(mesh->mFaces[i].mIndices[j]);
    }

    unsigned int VAO, VBO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    int stride = 6 * sizeof(float);
    
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void*)0);
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    return {VAO, VBO, EBO, indices.size()};
}
