#version 330 core

in vec3 FragPos;
in vec3 Normal;
in vec3 viewPos;

out vec4 FragColor;

void main()
{

  //vec3 viewPos = vec3(-19., 0., 1.);
  vec3 lightPos = vec3(-30., 0., 1.);
    vec3 color = vec3(0.2, 0.2, 0.2);
    // ambient
    vec3 ambient = 0.15 * color;
    // diffuse
    vec3 lightDir = normalize(lightPos - FragPos);
    vec3 normal = normalize(Normal);
    float diff = max(dot(lightDir, normal), 0.0);
    vec3 diffuse = diff * color;
    // specular
    vec3 viewDir = normalize(viewPos - FragPos);
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = 0.0;
    bool blinn = false;
    if(blinn)
    {
        vec3 halfwayDir = normalize(lightDir + viewDir);  
        spec = pow(max(dot(normal, halfwayDir), 0.0), 32.0);
    }
    else
    {
        vec3 reflectDir = reflect(-lightDir, normal);
        spec = pow(max(dot(viewDir, reflectDir), 0.0), 8.0);
    }
    vec3 specular = vec3(0.3) * spec; // assuming bright white light color
    FragColor = vec4(ambient + diffuse + specular, 1.0);
}

//void main() {
//    FragColor = vec4(1.0, 0., 0., 1.0);
//}
