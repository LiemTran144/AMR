// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from rosbasic_move_msgs:srv/Control.idl
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
#include "rosbasic_move_msgs/srv/detail/control__struct.h"
#include "rosbasic_move_msgs/srv/detail/control__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool rosbasic_move_msgs__srv__control__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[48];
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
    assert(strncmp("rosbasic_move_msgs.srv._control.Control_Request", full_classname_dest, 47) == 0);
  }
  rosbasic_move_msgs__srv__Control_Request * ros_message = _ros_message;
  {  // quangduong
    PyObject * field = PyObject_GetAttrString(_pymsg, "quangduong");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->quangduong = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // khoangcach
    PyObject * field = PyObject_GetAttrString(_pymsg, "khoangcach");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->khoangcach = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * rosbasic_move_msgs__srv__control__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Control_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rosbasic_move_msgs.srv._control");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Control_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  rosbasic_move_msgs__srv__Control_Request * ros_message = (rosbasic_move_msgs__srv__Control_Request *)raw_ros_message;
  {  // quangduong
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->quangduong);
    {
      int rc = PyObject_SetAttrString(_pymessage, "quangduong", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // khoangcach
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->khoangcach);
    {
      int rc = PyObject_SetAttrString(_pymessage, "khoangcach", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "rosbasic_move_msgs/srv/detail/control__struct.h"
// already included above
// #include "rosbasic_move_msgs/srv/detail/control__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool rosbasic_move_msgs__srv__control__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[49];
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
    assert(strncmp("rosbasic_move_msgs.srv._control.Control_Response", full_classname_dest, 48) == 0);
  }
  rosbasic_move_msgs__srv__Control_Response * ros_message = _ros_message;
  {  // success
    PyObject * field = PyObject_GetAttrString(_pymsg, "success");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->success = (Py_True == field);
    Py_DECREF(field);
  }
  {  // notification
    PyObject * field = PyObject_GetAttrString(_pymsg, "notification");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->notification, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * rosbasic_move_msgs__srv__control__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Control_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rosbasic_move_msgs.srv._control");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Control_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  rosbasic_move_msgs__srv__Control_Response * ros_message = (rosbasic_move_msgs__srv__Control_Response *)raw_ros_message;
  {  // success
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->success ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "success", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // notification
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->notification.data,
      strlen(ros_message->notification.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "notification", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
