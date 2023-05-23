# Copyright 2018-2021 NXP
# Released under the MIT license (see COPYING.MIT for the terms)

DESCRIPTION = "This is ai camera image"

require recipes-fsl/images/imx-image-core.bb

APPS_PACKAGE += "\
    cam-depth \
    apps-uvc \
"
		
IMAGE_INSTALL += " \
    packagegroup-imx-isp \
	packagegroup-fsl-gstreamer1.0 \
	packagegroup-fsl-gstreamer1.0-full \
	packagegroup-imx-ml \
	v4l-utils \
	${APPS_PACKAGE} \
"

export IMAGE_BASENAME = "imx-image-lgaicam"

