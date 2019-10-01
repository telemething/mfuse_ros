#pragma once

//#define USING_WINDOWS

#ifdef USING_WINDOWS

#include <Windows.h>
#include <dshow.h>

#pragma comment(lib, "strmiids")

#include <map>
#include <string>

/*struct Device {
	int id; // This can be used to open the device in OpenCV
	std::string devicePath;
	std::string deviceName; // This can be used to show the devices to the user
};*/

class DeviceEnumerator {

public:

	struct Device {
		int id; // This can be used to open the device in OpenCV
		std::string devicePath;
		std::string deviceName; // This can be used to show the devices to the user
	};

	DeviceEnumerator() = default;
	std::map<int, Device> getDevicesMap(const GUID deviceClass);
	std::map<int, Device> getVideoDevicesMap();
	std::map<int, Device> getAudioDevicesMap();

private:

	std::string ConvertBSTRToMBS(BSTR bstr);
	std::string ConvertWCSToMBS(const wchar_t* pstr, long wslen);

};

#else

#include <string>
#include <vector>
#include <map>
#include "opencv2/opencv.hpp"

class GUID
{
  public:

    // create a guid from vector of bytes
    GUID(const std::vector<unsigned char> &bytes){};

    // create a guid from array of bytes
    GUID(const unsigned char *bytes){};

    // create a guid from string
    GUID(const std::string &fromString){};

    // create empty guid
    GUID(){};

    // copy constructor
    GUID(const GUID &other){};

    // overload assignment operator
    GUID &operator=(const GUID &other);

    // overload equality and inequality operator
    bool operator==(const GUID &other) const;
    bool operator!=(const GUID &other) const;

  private:

    // actual data
    std::vector<unsigned char> _bytes;

    // make the << operator a friend so it can access _bytes
    //friend ostream &operator<<(ostream &s, const GUID &guid);
};

class DeviceEnumerator {

public:

	GUID CLSID_VideoInputDeviceCategory;
	GUID CLSID_AudioInputDeviceCategory;

	struct Device {
		int id; // This can be used to open the device in OpenCV
		std::string devicePath;
		std::string deviceName; // This can be used to show the devices to the user
	};

	DeviceEnumerator() = default;
	std::map<int, Device> getDevicesMap(const GUID deviceClass);
	std::map<int, Device> getVideoDevicesMap();
	std::map<int, Device> getAudioDevicesMap();

private:

	//std::string ConvertBSTRToMBS(BSTR bstr);
	//std::string ConvertWCSToMBS(const wchar_t* pstr, long wslen);

};

#endif //USING_WINDOWS
