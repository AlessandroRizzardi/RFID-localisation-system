# RFID Localization System in Mobile Robotics: A Multi-Hypothesis Kalman Filter Approach with Distributed WLS Algorithm
This project was developed by Elia Bontempelli and Alessandro Rizzardi, master's students at Mechatronics Engineering in Trento, for "Intelligent Distributed Systems" course

## Abstract
Indoor localization estimation is a research area that has drawn the attention of many scholars in recent years. 
The key difficulty nowadays is to develop a robust estimating method based on low-cost sensors. 
A Radio Frequency IDentification (RFID) system is proposed for this purpose, despite the fact that RFID signals can’t provide a direct measurement of the distance between the reader and the tag. 
A Multi-Hypothesis Extended Kalman Filter was used in this study to handle the issue of RFID system phase ambiguity, in order to compute the distance tag-reader and obtain a good localization of the tag in a wide unknown region. 
In addition, a distributed WLS algorithm is developed to increase estimation performance.

## Introduction
Radio Frequency IDentification (RFID) systems have
emerged as a pivotal technology in diverse applications,
particularly in the fields of item tracking and indoor
localization. RFID devices are a preferred choice over
alternative technologies with equivalent functionality because
to their low cost, low maintenance requirements, and small
size. This work focuses on a novel aspect of RFID system
deployment in which the tag remains stationary within a
somewhat large environment while the mobile robot, on
which the reader is mounted, attempts to find and localise it.

The reader sends out an RF signal, which activates passive
RFID tags in its vicinity. When an RFID tag receives the
reader’s signal, it responds by transmitting its data to the
reader.
R. Miesen et al. (2011) [1] describe different techniques
used to compute the distance between the two devices, such
as the received signal strength indicator, the measure of time
of arrival or the angle of arrival, or the measurement of the
phase of the backscattered signal. The latter technology is the
one used for this project.
Despite their prevalence and utility, RFID systems encounter
challenges, especially when deployed in environments where
precise localization is crucial. Phase-based techniques suffer
from ambiguities in distance (location) estimation. Distances
which differ by a multiple of λ/2 (where λ is the wavelength
of the signal), give rise to the same phase reading. This
ambiguity in the phase measurement make impossible to
directly recover the distance between the tag and the reader.
In addressing the challenge at hand, a Multi-Hypothesis
Extended Kalman Filter is implemented, similar to the one
presented by E. DiGiampaolo and F. Martinelli [3]. The
basic idea is to exploit the functionalities of the Kalman
Filter, fusing the wheel encoder readings with the RFID
signals and then create a certain number of EKF instances,
each one initialized on a different cycle corresponding to
the initial phase measurement. Ultimately, one of these
different instances is selected, associated with an estimate
of the distance between the robot and the tag. Actually the
approach, while estimating the tag-reader distance, allows to
obtain also the bearing of the tag with respect to the reader,
i.e., it allows to estimate the relative position of the tag with
respect to the robot.

In the second phase of the project, the approach involves
tackling the issue in a distributed fashion, with the primary
objective of enhancing tag localization precision. The RFID
readers are mounted on multiple robots, and that each
exchanges information with the others about its own estimate
via a distributed WLS algorithm.
