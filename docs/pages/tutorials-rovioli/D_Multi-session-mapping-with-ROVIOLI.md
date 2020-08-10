## Multi-session mapping with ROVIOLI

### Creating a multisession map

ROVIOLI can be used as a frontend to create multisession maps, i.e. maps constructed using multiple single recordings (often called missions within the maplab framework). Creating such maps is very simple and consists of two steps:
1. Build separate maps using ROVIOLI and store them as VI-maps: [Running ROVIOLI in VIO mode](Running-ROVIOLI-in-VIO-mode)
2. Merge the maps and jointly refine them using the offline maplab console: [Preparing a multisession map](Preparing-a-multi-session-map)



### Localizing against a multisession map

You can create a localization map out of a VI-map using the following command within the maplab console:
```
generate_summary_map_and_save_to_disk --summary_map_save_path your_summary_map
```

The map saved in ``your_summary_map`` can then be used by ROVIOLI to localize, as described in [ROVIOLI Localization Mode](Running-ROVIOLI-in-Localization-mode) tutorial.

### Extending a multisession map

If you would like to include the new mission in your multisession map, you should load both the multisession map and the new map and just follow the [Multisession mapping tutorial](Preparing-a-multi-session-map).
