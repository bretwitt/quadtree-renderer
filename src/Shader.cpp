// Shader.cpp
#include "Shader.h"
#include <fstream>
#include <sstream>
#include <iostream>

Shader::Shader(const char* vertexPath, const char* fragmentPath)
{
    // 1. Retrieve the vertex/fragment source code from filePath
    std::string vertexCode;
    std::string fragmentCode;
    std::ifstream vShaderFile;
    std::ifstream fShaderFile;
    
    // Ensure ifstream objects can throw exceptions
    vShaderFile.exceptions (std::ifstream::failbit | std::ifstream::badbit);
    fShaderFile.exceptions (std::ifstream::failbit | std::ifstream::badbit);
    
    try 
    {
        vShaderFile.open(vertexPath);
        fShaderFile.open(fragmentPath);
        
        std::stringstream vShaderStream, fShaderStream;
        
        // Read file's buffer contents into streams
        vShaderStream << vShaderFile.rdbuf();
        fShaderStream << fShaderFile.rdbuf();		

        // Close file handlers
        vShaderFile.close();
        fShaderFile.close();

        // Convert stream into string
        vertexCode   = vShaderStream.str();
        fragmentCode = fShaderStream.str();			
    }
    catch (std::ifstream::failure& e)
    {
        std::cerr << "ERROR::SHADER::FILE_NOT_SUCCESSFULLY_READ\n";
    }
    
    const char* vShaderCode = vertexCode.c_str();
    const char * fShaderCode = fragmentCode.c_str();
    
    // 2. Compile shaders
    unsigned int vertex, fragment;
    int success;
    char infoLog[512];
    
    // Vertex Shader
    vertex = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex, 1, &vShaderCode, NULL);
    glCompileShader(vertex);
    
    // Print compile errors if any
    glGetShaderiv(vertex, GL_COMPILE_STATUS, &success);
    if(!success)
    {
        glGetShaderInfoLog(vertex, 512, NULL, infoLog);
        std::cerr << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" 
                  << infoLog << "\n";
    };
    
    // Fragment Shader
    fragment = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment, 1, &fShaderCode, NULL);
    glCompileShader(fragment);
    
    // Print compile errors if any
    glGetShaderiv(fragment, GL_COMPILE_STATUS, &success);
    if(!success)
    {
        glGetShaderInfoLog(fragment, 512, NULL, infoLog);
        std::cerr << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" 
                  << infoLog << "\n";
    };
    
    // Shader Program
    ID = glCreateProgram();
    glAttachShader(ID, vertex);
    glAttachShader(ID, fragment);
    glLinkProgram(ID);
    
    // Print linking errors if any
    glGetProgramiv(ID, GL_LINK_STATUS, &success);
    if(!success)
    {
        glGetProgramInfoLog(ID, 512, NULL, infoLog);
        std::cerr << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" 
                  << infoLog << "\n";
    }
    
    // Delete the shaders as they're linked into our program now and no longer necessary
    glDeleteShader(vertex);
    glDeleteShader(fragment);
}

void Shader::use() const
{ 
    glUseProgram(ID); 
}

void Shader::setBool(const std::string &name, bool value) const
{         
    glUniform1i(glGetUniformLocation(ID, name.c_str()), (int)value); 
}

void Shader::setInt(const std::string &name, int value) const
{ 
    glUniform1i(glGetUniformLocation(ID, name.c_str()), value); 
}

void Shader::setFloat(const std::string &name, float value) const
{ 
    glUniform1f(glGetUniformLocation(ID, name.c_str()), value); 
}

void Shader::setMat4(const std::string &name, const glm::mat4 &mat) const
{
    glUniformMatrix4fv(glGetUniformLocation(ID, name.c_str()), 
                       1, GL_FALSE, &mat[0][0]);
}

void Shader::setVec3(const std::string &name, const glm::vec3 &vec) const
{
    glUniform3fv(glGetUniformLocation(ID, name.c_str()), 1, &vec[0]);
}