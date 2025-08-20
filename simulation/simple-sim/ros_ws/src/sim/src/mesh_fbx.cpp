#include "mesh_fbx.h"
#include <stb_image.h>

unsigned int FBXMesh::loadTexture(const std::string& p){
    int w,h,c; stbi_set_flip_vertically_on_load(true);
    unsigned char* d = stbi_load(p.c_str(), &w, &h, &c, 0);
    if(!d) { std::cerr<<"Texture load failed: "<<p<<"\n"; return 0; }
    GLenum fmt = c == 3 ? GL_RGB : GL_RGBA;
    unsigned int tex; glGenTextures(1,&tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexImage2D(GL_TEXTURE_2D,0,fmt,w,h,0,fmt,GL_UNSIGNED_BYTE,d);
    glGenerateMipmap(GL_TEXTURE_2D); stbi_image_free(d);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    return tex;
}


Mesh FBXMesh::loadMesh(const aiMesh* m, const aiScene* scene, std::string base_path){
    std::vector<float> verts;
    std::vector<unsigned int> idx;

    std::cout << "loading mesh" << std::endl;

    for(unsigned i=0;i<m->mNumVertices;i++){
        verts.insert(verts.end(), {
            m->mVertices[i].x * scale + x_offset,
	    m->mVertices[i].y * scale + y_offset,
	    m->mVertices[i].z * scale + z_offset,
            m->mTextureCoords[0] ? m->mTextureCoords[0][i].x : 0.0f,
            m->mTextureCoords[0] ? m->mTextureCoords[0][i].y : 0.0f
        });
    }
    for(unsigned i=0;i<m->mNumFaces;i++){
        for(unsigned j=0;j<m->mFaces[i].mNumIndices;j++)
            idx.push_back(m->mFaces[i].mIndices[j]);
    }

    unsigned int vao,vbo,ebo;
    glGenVertexArrays(1,&vao); glGenBuffers(1,&vbo); glGenBuffers(1,&ebo);
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER,vbo);
    glBufferData(GL_ARRAY_BUFFER, verts.size()*sizeof(float), verts.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, idx.size()*sizeof(unsigned int), idx.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,5*sizeof(float),(void*)0); glEnableVertexAttribArray(0);
    glVertexAttribPointer(1,2,GL_FLOAT,GL_FALSE,5*sizeof(float),(void*)(3*sizeof(float))); glEnableVertexAttribArray(1);

    Mesh mesh{vao,vbo,ebo,idx.size(),0};

    auto start = std::chrono::high_resolution_clock::now();
    if(m->mMaterialIndex>=0){
        aiMaterial* mat = scene->mMaterials[m->mMaterialIndex];
        aiString path;
        if(mat->GetTexture(aiTextureType_DIFFUSE,0,&path)==AI_SUCCESS){
            std::string p = path.C_Str();
            mesh.tex = loadTexture(base_path + "/" + p);
        }
    }
    double elapsed_seconds = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
    std::cout << "texture load time: " << elapsed_seconds << " seconds" << std::endl;
    
    std::cout << "done loading mesh" << std::endl;
    
    return mesh;
}

FBXMesh::FBXMesh(std::string filename, float scale, float x_offset, float y_offset, float z_offset)
  : scale(scale)
  , x_offset(x_offset)
  , y_offset(y_offset)
  , z_offset(z_offset){
  std::string base_path = std::filesystem::path(filename).parent_path().string();
  Assimp::Importer importer;
  const aiScene* scene = importer.ReadFile(filename,
					   aiProcess_Triangulate|aiProcess_PreTransformVertices);
  if(!scene || !scene->HasMeshes()){
    std::cerr<<"FBX load failed\n";
    return;
  }
  
  for(unsigned i=0;i<scene->mNumMeshes;i++){
    meshes.push_back(loadMesh(scene->mMeshes[i],scene, base_path));
  }
}

void FBXMesh::draw(){
  for(auto& m : meshes){
    glBindVertexArray(m.VAO);
    if(m.tex){
      glActiveTexture(GL_TEXTURE0);
      glBindTexture(GL_TEXTURE_2D,m.tex);
    }
    glDrawElements(GL_TRIANGLES, m.indexCount, GL_UNSIGNED_INT, 0);
  }
}
