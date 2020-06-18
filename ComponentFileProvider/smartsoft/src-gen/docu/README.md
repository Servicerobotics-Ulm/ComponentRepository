<!--- This file is generated from the ComponentFileProvider.componentDocumentation model --->
<!--- do not modify this file manually as it will by automatically overwritten by the code generator, modify the model instead and re-generate this file --->

# ComponentFileProvider Component

![ComponentFileProvider-ComponentImage](model/ComponentFileProviderComponentDefinition.jpg)


| Metaelement | Documentation |
|-------------|---------------|
| License |  |
| Hardware Requirements |  |
| Purpose |  |



## Service Ports

### FileReadQueryAnsw

An query send to this service will trigger the read of the queried file from the filesystem. 
						In case the file could not be read the query answer will be invalid.
                        The filename will be interpreted as is, including the path of the file.

### FileWriteQueryAnsw

An query send to this service will trigger the write of the queried file to the filesystem. 
						In case the file could not be written the query answer will be invalid.
						The filename will be interpreted as is, including the path of the file


## Component Parameters ComponentFileProviderParams

