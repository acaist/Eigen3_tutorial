/*
#include "eigen_examples.h"
#include <iostream>

#include <Python.h>
#include <opencv2/opencv.hpp>

void opencv2test()
{
   cv::Mat image(300, 400, CV_8UC3, cv::Scalar(0, 255, 0));

   cv::imshow("Green Image", image);
   cv::waitKey(0);

}

#include <Python.h>

int embedPython() {
    // Initialize the Python interpreter
    Py_Initialize();

    // Import the necessary Python modules
    PyObject* matplotlib = PyImport_ImportModule("matplotlib.pyplot");

    // Check for errors during the module import
    if (!matplotlib) {
        PyErr_Print();
        return 1;
    }

    // Create a data matrix in C++
    double data[3][3] = {
        {1.0, 2.0, 3.0},
        {4.0, 5.0, 6.0},
        {7.0, 8.0, 9.0}
    };

    // Convert the C++ data matrix to a Python object
    PyObject* pyData = PyList_New(3);
    for (int i = 0; i < 3; ++i) {
        PyObject* row = PyList_New(3);
        for (int j = 0; j < 3; ++j) {
            PyObject* value = PyFloat_FromDouble(data[i][j]);
            PyList_SetItem(row, j, value);
        }
        PyList_SetItem(pyData, i, row);
    }

    // Call Python code to plot the figure
    PyObject* plotFunc = PyObject_GetAttrString(matplotlib, "imshow");
    PyObject* args = PyTuple_Pack(1, pyData);
    PyObject* kwargs = PyDict_New();
    PyObject* result = PyObject_Call(plotFunc, args, kwargs);

    // Check for errors during the function call
    if (!result) {
        PyErr_Print();
        return 1;
    }

    // Call the show() function to display the plot
    PyObject* showFunc = PyObject_GetAttrString(matplotlib, "show");
    PyObject* showArgs = PyTuple_New(0);
    PyObject* showResult = PyObject_CallObject(showFunc, showArgs);

    // Cleanup and release resources
    Py_DECREF(pyData);
    Py_DECREF(plotFunc);
    Py_DECREF(args);
    Py_DECREF(kwargs);
    Py_DECREF(result);
    Py_DECREF(matplotlib);

    // Finalize the Python interpreter
    Py_Finalize();

    return 0;
}
*/