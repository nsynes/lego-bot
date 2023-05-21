# lego-bot
Code to run a Raspberry Pi and Lego based remote controlled car/robot.

To automatically run the code when the lego-bot is turned on, you need to change the `rc.local` file. In the terminal, run:

```
sudo nano /etc/rc.local
```

Then, in the `rc.local` file, enter the following line of code before the `exit 0` line:

```
python /path/to/file/lego-bot.py &
```
[lego-bot diagrams and instructions](lego-bot.pdf)
![lego-bot diagram](/img/lego-bot.png "lego-bot diagram")
