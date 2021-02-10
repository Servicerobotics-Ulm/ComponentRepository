Steps to update/generate code to parse the XSD for the objects xml:
1.: xsd cxx-tree --generate-serialization --generate-detach openRaveObjectsDB.xsd
2.: rename openRaveObjectsDB.cxx -> openRaveObjectsDB.cc
3.: !!!NOTE!!! make sure to have a copy of the xsd file in the same folder as the openRaveObjectsDB.xml. Alternatively: reference the xsd file in the header of the xml file correctly