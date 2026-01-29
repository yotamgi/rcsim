#version 100

precision highp float;

// Input vertex attributes (from vertex shader)
varying vec3 fragPosition;
varying vec2 fragTexCoord;
varying vec4 fragColor;
varying vec3 fragNormal;

// Input uniform values
uniform sampler2D texture0;
uniform vec4 colDiffuse;

// NOTE: Add your custom variables here

#define     MAX_LIGHTS              4
#define     LIGHT_DIRECTIONAL       0
#define     LIGHT_POINT             1

struct Light {
    int enabled;
    int type;
    vec3 position;
    vec3 target;
    vec4 color;
};

// Input lighting values
uniform Light lights[MAX_LIGHTS];
uniform vec4 ambient;
uniform vec3 viewPos;

// Shadowmap inputs.
uniform int useShadow;
uniform mat4 lightVP; // Light source view-projection matrix
uniform sampler2D shadowMap;
uniform int shadowMapResolution;

void main()
{
    // Texel color fetching from texture sampler
    vec4 texelColor = texture2D(texture0, fragTexCoord);
    vec3 lightDot = vec3(0.0);
    vec3 normal = normalize(fragNormal);
    vec3 viewD = normalize(viewPos - fragPosition);
    vec3 specular = vec3(0.0);

    vec4 tint = colDiffuse*fragColor;

    // NOTE: Implement here your fragment shader code

    for (int i = 0; i < MAX_LIGHTS; i++)
    {
        if (lights[i].enabled == 1)
        {
            vec3 light = vec3(0.0);

            if (lights[i].type == LIGHT_DIRECTIONAL)
            {
                light = -normalize(lights[i].target - lights[i].position);
            }

            if (lights[i].type == LIGHT_POINT)
            {
                light = normalize(lights[i].position - fragPosition);
            }

            float NdotL = max(dot(normal, light), 0.0);

            float specCo = 0.0;
            if (NdotL > 0.0) specCo = pow(max(0.0, dot(viewD, reflect(-(light), normal))), 16.0); // 16 refers to shine

            // For the first light, use the shadow.
            if ((i == 0) && (useShadow != 0)) {
                vec4 fragPosLightSpace = lightVP*vec4(fragPosition, 1);
                fragPosLightSpace.xyz /= fragPosLightSpace.w; // Perform the perspective division
                fragPosLightSpace.xyz = (fragPosLightSpace.xyz + 1.0)/2.0; // Transform from [-1, 1] range to [0, 1] range
                vec2 sampleCoords = fragPosLightSpace.xy;
                float curDepth = fragPosLightSpace.z;

                // Slope-scale depth bias: depth biasing reduces "shadow acne" artifacts, where dark stripes appear all over the scene
                // The solution is adding a small bias to the depth
                // In this case, the bias is proportional to the slope of the surface, relative to the light
                float bias = max(0.0008*(1.0 - dot(normal, light)), 0.00008);
                int shadowCounter = 0;
                const int numSamples = 9;

                // PCF (percentage-closer filtering) algorithm:
                // Instead of testing if just one point is closer to the current point,
                // we test the surrounding points as well
                // This blurs shadow edges, hiding aliasing artifacts
                vec2 texelSize = vec2(1.0/float(shadowMapResolution));
                for (int x = -1; x <= 1; x++)
                {
                    for (int y = -1; y <= 1; y++)
                    {
                        float sampleDepth = texture2D(shadowMap, sampleCoords + texelSize*vec2(x, y)).r;
                        if (curDepth - bias > sampleDepth) shadowCounter++;
                    }
                }
                float shadow_intensity = 0.7 * float(shadowCounter)/float(numSamples);
                lightDot += mix(lights[i].color.rgb*NdotL, vec3(0, 0, 0), shadow_intensity);
                specular += mix(vec3(specCo, specCo, specCo), vec3(0, 0, 0), shadow_intensity);
            } else {
                lightDot += lights[i].color.rgb*NdotL;
                specular += specCo;
            }
        }
    }

    vec4 finalColor = (texelColor*((tint + vec4(specular, 1.0))*vec4(lightDot, 1.0)));
    finalColor += texelColor*(ambient/10.0);

    // Gamma correction
    gl_FragColor = pow(finalColor, vec4(1.0/2.2));
}
