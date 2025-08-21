#pragma once
#include <string>
#include <glad/glad.h> 
GLuint createShaderProgram(const std::string& vertexPath, const std::string& fragmentPath);
