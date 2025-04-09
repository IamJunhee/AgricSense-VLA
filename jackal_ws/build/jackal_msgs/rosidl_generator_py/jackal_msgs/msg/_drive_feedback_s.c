// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from jackal_msgs:msg/DriveFeedback.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "jackal_msgs/msg/detail/drive_feedback__struct.h"
#include "jackal_msgs/msg/detail/drive_feedback__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool jackal_msgs__msg__drive_feedback__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[46];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("jackal_msgs.msg._drive_feedback.DriveFeedback", full_classname_dest, 45) == 0);
  }
  jackal_msgs__msg__DriveFeedback * ros_message = _ros_message;
  {  // current
    PyObject * field = PyObject_GetAttrString(_pymsg, "current");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->current = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // duty_cycle
    PyObject * field = PyObject_GetAttrString(_pymsg, "duty_cycle");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->duty_cycle = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // bridge_temperature
    PyObject * field = PyObject_GetAttrString(_pymsg, "bridge_temperature");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->bridge_temperature = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // motor_temperature
    PyObject * field = PyObject_GetAttrString(_pymsg, "motor_temperature");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->motor_temperature = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // measured_velocity
    PyObject * field = PyObject_GetAttrString(_pymsg, "measured_velocity");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->measured_velocity = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // measured_travel
    PyObject * field = PyObject_GetAttrString(_pymsg, "measured_travel");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->measured_travel = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // driver_fault
    PyObject * field = PyObject_GetAttrString(_pymsg, "driver_fault");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->driver_fault = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * jackal_msgs__msg__drive_feedback__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of DriveFeedback */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("jackal_msgs.msg._drive_feedback");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "DriveFeedback");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  jackal_msgs__msg__DriveFeedback * ros_message = (jackal_msgs__msg__DriveFeedback *)raw_ros_message;
  {  // current
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->current);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // duty_cycle
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->duty_cycle);
    {
      int rc = PyObject_SetAttrString(_pymessage, "duty_cycle", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // bridge_temperature
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->bridge_temperature);
    {
      int rc = PyObject_SetAttrString(_pymessage, "bridge_temperature", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motor_temperature
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->motor_temperature);
    {
      int rc = PyObject_SetAttrString(_pymessage, "motor_temperature", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // measured_velocity
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->measured_velocity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "measured_velocity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // measured_travel
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->measured_travel);
    {
      int rc = PyObject_SetAttrString(_pymessage, "measured_travel", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // driver_fault
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->driver_fault ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "driver_fault", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
