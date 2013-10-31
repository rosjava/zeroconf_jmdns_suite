^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package zeroconf_jmdns_suite
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.13 (2013-11-01)
------------------
* constrain open ranged dependencies.

0.1.12 (2013-10-31)
------------------
* use ROS_MAVEN_REPOSITORY

0.1.11 (2013-10-25)
------------------
* official maven style open ended dependencies.

0.1.10 (2013-09-23)
-------------------
* use updated ros gradle plugins with maven-publish for publishing.

0.1.9 (2013-09-22)
------------------
* removing osgi information.

0.1.8 (2013-09-17)
------------------
* bugfix install rule target name.
* depend on rosjava_bootstrap for gradle plugins.
* gradle wrapper 1.7
* use catkin package xml version information
* gradled, maven'd, deb'd.
* ros-java -> rosjava migration.
* refactoring for unique java package names.
* fix badly copied package name for jdmns tutorial.
* catkinization using rosjava_tools
* demos working with new gradle build.
* cleared out non jmdns packages.

0.1.7 (2013-04-29)
------------------
* point at legacy rosinstallers.
* using multiproject gradle now so we can add more projects here.
* rosinstall file for rosjava.
* bugfix for name in package.
* readme added.
* now building and installing with catkin.
* first commit pulled in from kazuto's hg clone.
* avahi and comms moved out.

0.1.6 (2012-07-17)
------------------
* bugfix version bump.
* refactored for ros code style format.
* bugfix bad array size check, closes `#35 <https://github.com/rosjava/zeroconf_jmdns_suite/issues/35>`_.
* rosinstaller, use https, not git.
* updated fuerte rosinstaller.
* zeroconf: updated the rosinstaller for fuerte.

0.1.5 (2012-05-24)
------------------
* updated rosdeps for rosdep2.

0.1.4 (2012-04-12)
------------------
* Adjusting cmakelists for release into fuerte.
* hoping to avert compilation of jmdns. Probably should divert to a separate stack.
* new rosdeps.

0.1.3 (2012-01-04)
------------------
* jmdns : better getInstance call.
* jmdns: hack new jmmdns getInstance to always create, hack to trigger serviceRemoved callbacks on network down.
* jmdns_demos : forgot to add the publisher class.
* zeroconf_jmdns : added display names to the network interface logging.
* jmdns : put default listener before the listener instantiation (sometimes add services will come in before it gets set otherwise).
* zeroconf_jmdns : default listener to default discovery callback.
* zeroconf_jmdns : now hiding all service_info api.
* zeroconf_implementations : fix mediawiki formatting.
* zeroconf_avahi : fix code after update to zeroconf_comms ip4/ipv6 upgrade.
* zeroconf_jmdns: ros api utility toString function.
* zeroconf: split the ros api to include both ipv4 and ipv6, the jmdns is now working with that, but jmdns needs further updates to convert fully over to the ros comms representations.
* zeroconf_jmdns : now working rather more nicely, less multi interface confusion, but going to have to update everything else.
* this will break everything (new comms), but jmdns is looking better.
* zeroconf_jmdns_demos : better output.
* zeroconf_jmdns_demos : minor demo program refactoring.
* zeroconf_implementations : make sure we get resolving happening if a new service is found.
* zeroconf_jmdns_demos : master browser running, but strange resolving behaviour seen by jmdns.
* zeroconf_jmdns : added a default listener callback and api.
* zeroconf_jmdns : listeners with userland callback handlers.
* zeroconf_jmdns : user listener callbacks.
* zeroconf_jmdns : toString support for the demo.
* zeroconf_implementations : loggers so I can delete duplicating code in jmdns and android_jmdns
* zeroconf_jmdns : comment some debug spam.
* zeroconf_jmdns: simplify dependencies.
* zeroconf_implementations: forgot to update the build to the standard ros template.
* zeroconf_implementations : upgrade jmdns build method, fix android in rosinstalls.
* zeroconf_implementations : finalise the zeroconf_overlay.rosinstall.
* zeroconf: new overlay installer.

0.1.2 (2011-12-19)
------------------
* zeroconf: only maintain one rosinstaller, also bypass jmdns builds if rosjava_core 0.1.0 (old) is present.
* zeroconf_jmdns_demos : cleanup.
* zeroconf_implementations : decided to split the jmdns stack again (for illustration).



