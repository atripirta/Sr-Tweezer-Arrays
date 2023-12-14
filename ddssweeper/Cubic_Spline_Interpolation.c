#include <stdio.h>
#include <stdlib.h>

// Define the maximum number of data points
#define MAX_POINTS 13

// Function to perform cubic spline interpolation
void cubicSplineInterpolation(float x[MAX_POINTS], float y[MAX_POINTS], int n, float resultX) {
    float h[MAX_POINTS - 1];
    float alpha[MAX_POINTS - 1];
    float l[MAX_POINTS];
    float mu[MAX_POINTS - 1];
    float z[MAX_POINTS];
    float c[MAX_POINTS];
    float b[MAX_POINTS];
    float d[MAX_POINTS];

    // Step 1: Compute h and alpha
    for (int i = 0; i < n - 1; i++) {
        h[i] = x[i + 1] - x[i];
        alpha[i] = (3.0 / h[i]) * (y[i + 1] - y[i]) - (3.0 / h[i - 1]) * (y[i] - y[i - 1]);
    }

    // Step 2: Compute l, mu, and z
    l[0] = 1.0;
    mu[0] = 0.0;
    z[0] = 0.0;

    for (int i = 1; i < n - 1; i++) {
        l[i] = 2.0 * (x[i + 1] - x[i - 1]) - h[i - 1] * mu[i - 1];
        mu[i] = h[i] / l[i];
        z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }

    l[n - 1] = 1.0;
    z[n - 1] = 0.0;
    c[n - 1] = 0.0;

    // Step 3: Back-substitution
    for (int j = n - 2; j >= 0; j--) {
        c[j] = z[j] - mu[j] * c[j + 1];
        b[j] = (y[j + 1] - y[j]) / h[j] - h[j] * (c[j + 1] + 2.0 * c[j]) / 3.0;
        d[j] = (c[j + 1] - c[j]) / (3.0 * h[j]);
    }

    // Step 4: Interpolation
    int interval = -1;
    for (int i = 0; i < n - 1; i++) {
        if (resultX >= x[i] && resultX <= x[i + 1]) {
            interval = i;
            break;
        }
    }

    if (interval == -1) {
        printf("Error: Data point outside the range of interpolation.\n");
        return;
    }

    // Evaluate the cubic spline at the desired point
    float deltaX = resultX - x[interval];
    float interpolationResult = y[interval] + b[interval] * deltaX + c[interval] * deltaX * deltaX + d[interval] * deltaX * deltaX * deltaX;
    printf("Interpolated value at x = %.2f: %.4f\n", resultX, interpolationResult);
    resultX=interpolationResult;
    
}

int main() {
    // Example data points
    float x[MAX_POINTS] ={85.5, 88.5, 91.5, 94.5, 97.5, 100.5, 103.5, 106.5, 109.5, 112.5, 115.5, 118.5, 121.50};
    float y[MAX_POINTS] = {0.695, 0.683, 0.698, 0.7, 0.74, 0.752, 0.743, 0.729, 0.732, 0.763, 0.831, 0.898, 0.948};

    // Number of data points
    int n = sizeof(x) / sizeof(x[0]);

    // Perform cubic spline interpolation at x = 7.5 (you can change this value)
    float resultX = 87.5;
    cubicSplineInterpolation(x, y, n, resultX);
    printf("\n%.2f",resultX);
    return 0;
}
