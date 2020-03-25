## Verbosity Policy

### List of verbosity categories

VLOG Level | Name        | Description
-----------|-------------|--------------------------------------------
0          | Silent      | No print output whatsoever.
1          | Basic       | Basic progress output at roughly 1/30Hz.
2          | Informative | Output more frequent than 1/30Hz. 
>2         | Debug       | Unregulated output for debugging purposes.

### Description
Level 1 should make clear to the user that the program is doing something (i.e. it does not hang). You can use a single-line text-based progress bar for processing large lists of elements. It limits the progress output to a single line!

Level 2 should inform the user about what is being done in the background and especially why things do or do not work. 

**Please note that every function in maplab should have level 1 output at 1/30Hz informing the user about the progress. 
Using LOG_INFO() is prohibited everywhere, except in code that is part of an executable only.** Penalty for violating this rule: http://helveticabold.tv/wordpress/wp-content/uploads/2010/03/09_fondue.jpg

You can start any application with verbosity level X like this: ```./application --v=X```
