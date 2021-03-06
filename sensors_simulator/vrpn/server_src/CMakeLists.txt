cmake_minimum_required(VERSION 2.6)

#-----------------------------------------------------------------------------
# XXX Things to make better.
#
# Move applications into subprojects in visual studio
# XXX tracker_to_poser
# XXX daq_server
# XXX vrpn_hid_device_watcher
# Fix the ones that are commented out so that they compile and run?

if(NOT VRPN_BUILD_SERVER_LIBRARY)
	message(FATAL_ERROR
		"Cannot compile servers without the full (server) library.  Enable it and try again.")
endif()

set(VRPN_SERVER_LIBRARY vrpnserver)

if(NOT WIN32)
	set(VRPN_ATMEL_LIBRARY vrpn_atmel)
endif()

#-----------------------------------------------------------------------------
# Phantom library (if configured)
# XXX Does not work with GHOST library
if(VRPN_USE_PHANTOM_SERVER AND OPENHAPTICS_FOUND)
	set(PHANTOM_SOURCES
		ghostEffects/InstantBuzzEffect.cpp
		buzzForceField.C
		constraint.C
		forcefield.C
		plane.C
		texture_plane.C
		trimesh.C
		vrpn_Phantom.C)

	set(PHANTOM_HEADERS
		ghostEffects/InstantBuzzEffect.h
		buzzForceField.h
		constraint.h
		forcefield.h
		ghost.h
		plane.h
		texture_plane.h
		trimesh.h
		vrpn_Phantom.h)
	include_directories(${OPENHAPTICS_INCLUDE_DIRS})
	include_directories(${VRPN_SOURCE_DIR}/server_src/ghostEffects)
	add_library(vrpn_phantom ${PHANTOM_SOURCES} ${PHANTOM_HEADERS})
	target_link_libraries(vrpn_phantom ${OPENHAPTICS_LIBRARIES})

	set_property(TARGET vrpn_phantom PROPERTY
		PUBLIC_HEADER ${PHANTOM_HEADERS})

	install(TARGETS vrpn_phantom
		ARCHIVE DESTINATION lib
		PUBLIC_HEADER DESTINATION include)
	set (VRPNPHANTOMLIB vrpn_phantom)
endif()

#-----------------------------------------------------------------------------
# DirectShow base-class library (if configured)
if(VRPN_BUILD_DIRECTSHOW_VIDEO_SERVER AND DIRECTSHOW_FOUND)
	add_definitions( -DCOINIT_DISABLE_OLE1DDE=0x4 )
	add_library(DirectShow_baseclasses
		${DirectShow_BASECLASS_DIR}/amextra.cpp
		${DirectShow_BASECLASS_DIR}/amfilter.cpp
		${DirectShow_BASECLASS_DIR}/amvideo.cpp
		${DirectShow_BASECLASS_DIR}/combase.cpp
		${DirectShow_BASECLASS_DIR}/cprop.cpp
		${DirectShow_BASECLASS_DIR}/ctlutil.cpp
		${DirectShow_BASECLASS_DIR}/ddmm.cpp
		${DirectShow_BASECLASS_DIR}/dllentry.cpp
		${DirectShow_BASECLASS_DIR}/dllsetup.cpp
		${DirectShow_BASECLASS_DIR}/mtype.cpp
		${DirectShow_BASECLASS_DIR}/outputq.cpp
		${DirectShow_BASECLASS_DIR}/pstream.cpp
		${DirectShow_BASECLASS_DIR}/pullpin.cpp
		${DirectShow_BASECLASS_DIR}/refclock.cpp
		${DirectShow_BASECLASS_DIR}/renbase.cpp
		${DirectShow_BASECLASS_DIR}/schedule.cpp
		${DirectShow_BASECLASS_DIR}/seekpt.cpp
		${DirectShow_BASECLASS_DIR}/source.cpp
		${DirectShow_BASECLASS_DIR}/strmctl.cpp
		${DirectShow_BASECLASS_DIR}/sysclock.cpp
		${DirectShow_BASECLASS_DIR}/transfrm.cpp
		${DirectShow_BASECLASS_DIR}/transip.cpp
		${DirectShow_BASECLASS_DIR}/videoctl.cpp
		${DirectShow_BASECLASS_DIR}/vtrans.cpp
		${DirectShow_BASECLASS_DIR}/winctrl.cpp
		${DirectShow_BASECLASS_DIR}/winutil.cpp
		${DirectShow_BASECLASS_DIR}/wxdebug.cpp
		${DirectShow_BASECLASS_DIR}/wxlist.cpp
		${DirectShow_BASECLASS_DIR}/wxutil.cpp)
	set(DirectShow_LIBS DirectShow_baseclasses)

	include_directories(${DirectShow_BASECLASS_DIR}
		${PLATFORM_SDK_INCLUDE_DIR}
		${PLATFORM_SDK_ATL_INCLUDE_DIR}
		${DIRECTX_SDK_INCLUDE_DIR})
	add_executable(directshow_video_server
		directshow_video_server/directx_video_imager_server.cpp
		directshow_video_server/directx_camera_server.cpp
		directshow_video_server/directx_camera_server.h)
	target_link_libraries(directshow_video_server
		${VRPN_SERVER_LIBRARY}
		${DirectShow_LIBS})
	set_target_properties(directshow_video_server
		PROPERTIES
		SOLUTION_FOLDER
		servers)
endif()

add_subdirectory(timecode_generator_server)

#-----------------------------------------------------------------------------
# Include directories needed
include_directories(${VRPN_SOURCE_DIR}/server_src
	${VRPN_SOURCE_DIR}/server_src/timecode_generator_server)

#-----------------------------------------------------------------------------
# Applications that need more than one file to run.
add_executable(vrpn_server
	vrpn.C
	vrpn_Generic_server_object.C
	vrpn_Generic_server_object.h)
target_link_libraries(vrpn_server
	${VRPN_SERVER_LIBRARY}
	vrpn_timecode_generator
	${VRPN_ATMEL_LIBRARY}
	${VRPNPHANTOMLIB})
set_target_properties(vrpn_server PROPERTIES SOLUTION_FOLDER servers)
set_property(TARGET
	vrpn_server
	PROPERTY
	PROJECT_LABEL
	"vrpn_server")
install(TARGETS vrpn_server
	RUNTIME DESTINATION bin)

#-----------------------------------------------------------------------------
# Install the vrpn.cfg file needed by the server.
install(FILES ${VRPN_SOURCE_DIR}/server_src/vrpn.cfg
	DESTINATION etc)


#-----------------------------------------------------------------------------
# Declaration of the simple test applications that don't have extra files

# XXX Are these files compiled somewhere?
#NIUtil.cpp
#NIUtil.h
#clock_server.C
#daq_server.C
#directXFF.C
#make_vrpn_libs_build.cpp
#nidaqClient.cpp
#nidaqServer.cpp



set(SRV_TEST_SOURCES
	client_and_server.C
	#forward.C
	#last_of_sequence.C
	#sample_analog.C
	#sample_server.C
	#testSharedObject.C
	test_analogfly.C
	test_auxiliary_logger.C
	test_freespace.C
	test_logging.C
	#test_mutex.C
	test_peerMutex.C
	test_radamec_spi.C
	test_rumble.C
	test_vrpn.C
	testimager_server.cpp
	#text.C
	#vrpn_HID_device_watcher.cpp
)

set(SRV_SERVER_SOURCES
        wiimote_head_tracker.C
)


###
# Tests
###
if(BUILD_TESTING)
	foreach(SOURCE ${SRV_TEST_SOURCES})
		get_filename_component(APP ${SOURCE} NAME_WE)
		add_executable(${APP} ${SOURCE})
		target_link_libraries(${APP} ${VRPN_SERVER_LIBRARY})
		set_target_properties(${APP} PROPERTIES SOLUTION_FOLDER tests)
		set_target_properties(${APP} PROPERTIES HAS_CXX yes)
		install(TARGETS ${APP}
			RUNTIME DESTINATION bin)
	endforeach()
endif()

###
# Servers
###
foreach(SOURCE ${SRV_SERVER_SOURCES})
        get_filename_component(APP ${SOURCE} NAME_WE)
        add_executable(${APP} ${SOURCE})
        target_link_libraries(${APP} ${VRPN_SERVER_LIBRARY})
        set_target_properties(${APP} PROPERTIES SOLUTION_FOLDER servers)
        set_target_properties(${APP} PROPERTIES HAS_CXX yes)
        install(TARGETS ${APP}
                RUNTIME DESTINATION bin)
endforeach()

###
# Install extra files
###

# Wiimote jconf files for fishtank VR
set(VRPN_WIIMOTE_JCONFS
        vrpn.wiimoteheadtracker.fishtankvr.1280x1024.jconf
        vrpn.wiimoteheadtracker.fishtankvr.1024x768.jconf
        vrpn.wiimoteheadtracker.fishtankvr.800x600.jconf)

install(FILES ${VRPN_WIIMOTE_JCONFS}
        DESTINATION share/vrpn-${CPACK_PACKAGE_VERSION}/vrjuggler-2.2-config)

