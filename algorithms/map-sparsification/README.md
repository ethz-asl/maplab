map_sparsification
=================

This package implements various methods for landmark selection in order to 
compress the maps to compact localization models. The landmarks are selected 
based on various utility criteria, such as the number of observations, number 
of datasets they were observed in etc. 

The basic heuristic algorithm and initial evaluations are presented in the
following paper. Consider citing it when using these results:
```
@inproceedings{dymczyk2015gist,
  title={The Gist of Maps -- Summarizing Experience for Lifelong Localization},
  author={Marcin Dymczyk and Lynen, Simon and Cieslewski, Titus and Bosse, Michael and Siegwart, Roland and Furgale, Paul},
  booktitle={Robotics and Automation (ICRA), 2015 IEEE International Conference on},
  pages={to appear},
  year={2015},
  organization={IEEE}
}
```

When using the ILP or QP landmark selection, please consider citing:
```
@inproceedings{dymczyk2015keep,
  title={Keep it brief: Scalable creation of compressed localization maps},
  author={Dymczyk, Marcin and Lynen, Simon and Bosse, Michael and Siegwart, Roland},
  booktitle={Intelligent Robots and Systems (IROS), 2015 IEEE/RSJ International Conference on},
  pages={2536--2542},
  year={2015},
  organization={IEEE}
}
```