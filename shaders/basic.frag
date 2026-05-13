#version 330 core
in vec3 vNormal;
in vec3 vWorldPos;
out vec4 FragColor;

uniform vec3 uColor;
uniform vec3 uLightDir;
uniform vec3 uViewPos;

void main() {
    vec3 N = normalize(vNormal);
    vec3 L = normalize(-uLightDir);
    float diff = max(dot(N, L), 0.0);
    float ambient = 0.25;

    vec3 V = normalize(uViewPos - vWorldPos);
    vec3 R = reflect(-L, N);
    float spec = pow(max(dot(V, R), 0.0), 32.0);

    vec3 col = uColor * (ambient + 0.75 * diff) + vec3(spec * 0.15);
    FragColor = vec4(col, 1.0);
}
