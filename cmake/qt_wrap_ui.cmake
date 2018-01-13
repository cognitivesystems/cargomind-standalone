# wrapper for QT_WRAP_UI to generate ui header files within src folder.
# For Ubuntu 12.04 do not set version string
# Ubunut 14.04: PCL 1.8

if(EXISTS "/etc/issue")
	file(READ "/etc/issue" LINUX_ISSUE)
	if(LINUX_ISSUE MATCHES "Ubuntu")
		string(REGEX MATCH "buntu ([0-9]+\\.[0-9]+)" UBUNTU "${LINUX_ISSUE}")
		set(LINUX_NAME "Ubuntu")        
		set(LINUX_VER "${CMAKE_MATCH_1}")		  
	endif(LINUX_ISSUE MATCHES "Ubuntu")
endif(EXISTS "/etc/issue")    

if (LINUX_VER MATCHES "14.04")
	MACRO (QT4_WRAP_UI_LOCAL outfiles )
	  QT4_EXTRACT_OPTIONS(ui_files ui_options ui_target ${ARGN})

	  FOREACH (it ${ui_files})
	    GET_FILENAME_COMPONENT(outfile ${it} NAME_WE)
	    GET_FILENAME_COMPONENT(infile ${it} ABSOLUTE)
	    SET(outfile ${PROJECT_SOURCE_DIR}/src/ui_${outfile}.h) # Here we set output
	    ADD_CUSTOM_COMMAND(OUTPUT ${outfile}
	      COMMAND ${QT_UIC_EXECUTABLE}
	      ARGS ${ui_options} -o ${outfile} ${infile}
	      MAIN_DEPENDENCY ${infile})
	    SET(${outfiles} ${${outfiles}} ${outfile})
	  ENDFOREACH (it)

	ENDMACRO (QT4_WRAP_UI_LOCAL)
else (LINUX_VER MATCHES "14.04")
	# ubuntu 12.04 mustn't have the ui_target parameter
	MACRO (QT4_WRAP_UI_LOCAL outfiles )
	  QT4_EXTRACT_OPTIONS(ui_files ui_options ${ARGN})

	  FOREACH (it ${ui_files})
	    GET_FILENAME_COMPONENT(outfile ${it} NAME_WE)
	    GET_FILENAME_COMPONENT(infile ${it} ABSOLUTE)
	    SET(outfile ${PROJECT_SOURCE_DIR}/src/ui_${outfile}.h) # Here we set output
	    ADD_CUSTOM_COMMAND(OUTPUT ${outfile}
	      COMMAND ${QT_UIC_EXECUTABLE}
	      ARGS ${ui_options} -o ${outfile} ${infile}
	      MAIN_DEPENDENCY ${infile})
	    SET(${outfiles} ${${outfiles}} ${outfile})
	  ENDFOREACH (it)

	ENDMACRO (QT4_WRAP_UI_LOCAL)
endif (LINUX_VER MATCHES "14.04")



