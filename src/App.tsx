import React, { useEffect } from 'react';
import './App.css';

import { createMuiTheme, makeStyles, ThemeProvider, Theme, createStyles } from '@material-ui/core/styles';

import AppBar from '@material-ui/core/AppBar';
import Container from '@material-ui/core/Container';
import Toolbar from '@material-ui/core/Toolbar';
import Typography from '@material-ui/core/Typography';
import Paper from '@material-ui/core/Paper';
import Grid from '@material-ui/core/Grid';

import Radio from '@material-ui/core/Radio';
import RadioGroup from '@material-ui/core/RadioGroup';
import FormControlLabel from '@material-ui/core/FormControlLabel';
import FormControl from '@material-ui/core/FormControl';
import Button from '@material-ui/core/Button';

// @ts-ignore
import Rowma from 'rowma_js';

const theme = createMuiTheme({
  palette: {
    primary: {
      main: '#ffffff',
    },
    secondary: {
      main: '#ED4A70',
    },
    contrastThreshold: 3,
    tonalOffset: 0.2,
  },
  typography: {
    button: {
      textTransform: 'none'
    }
  }
});

const useStyles = makeStyles((theme: Theme) => (
  createStyles({
    root: {
      flexGrow: 1,
    },
    menuButton: {
      marginRight: theme.spacing(2),
    },
    title: {
      flexGrow: 1,
    },
    paper: {
      padding: theme.spacing(2),
      textAlign: 'center',
      color: theme.palette.text.secondary,
    },
  })
));

const rowma = new Rowma({ baseURL: 'http://18.176.1.219' });

const App: React.FC = () => {
  const [robotUuids, setRobotUuids] = React.useState<Array<string> | undefined>(undefined);
  const [selectedRobot, setSelectedRobot] = React.useState<any | null>(null);
  const [rosrunCommands, setRosrunCommands] = React.useState<Array<string>>([]);
  const [selectedRosrunCommand, setSelectedRosrunCommand] = React.useState<string>('');
  const [roslaunchCommands, setRoslaunchCommands] = React.useState<Array<string>>([]);
  const [selectedRoslaunchCommand, setSelectedRoslaunchCommand] = React.useState<string>('');
  const [robot, setRobot] = React.useState<any>({});
  const [connectButtonColor, setConnectButtonColor] = React.useState<any>('primary');
  const [connectButtonText, setConnectButtonText] = React.useState<any>('Connect');

  const [socket, setSocket] = React.useState<any>(null);

  useEffect(() => {
    if (robotUuids === undefined) {
      rowma.currentConnectionList().then((res: any) => {
        console.log(res.data)
        setRobotUuids(res.data.map((robot: any) => robot.uuid));
      })
    }
  });

  const classes = useStyles();

  const handleRobotChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setSelectedRobot((event.target as HTMLInputElement).value);
  };

  const handleConnectClicked = () => {
    rowma.connect(selectedRobot).then((sock: any) => {
      setSocket(sock)
    }).catch((e: any) => {
      console.log(e)
    })

    rowma.getRobotStatus(selectedRobot).then((res: any) => {
      console.log(res.data)
      setRobot(res.data)
      setRosrunCommands(res.data['rosrunCommands']);
      setRoslaunchCommands(res.data['launchCommands']);
      setConnectButtonColor('secondary');
      setConnectButtonText('Disconnect');
    })
  }

  const handleRosrunCommandChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setSelectedRosrunCommand((event.target as HTMLInputElement).value);
  };

  const handleRoslaunchCommandChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setSelectedRoslaunchCommand((event.target as HTMLInputElement).value);
  };

  const handleRosrunButtonClick = () => {
    const rosrunArgs = '';
    rowma.runRosrun(socket, selectedRobot, selectedRosrunCommand, rosrunArgs);
  }

  const handleRoslaunchButtonClick = () => {
    rowma.runLaunch(socket, selectedRobot, selectedRoslaunchCommand)
  }

  return (
    <div className={`${classes.root} App`}>
      <ThemeProvider theme={theme}>
        <AppBar position="static">
          <Toolbar>
            <Container>
              <Typography variant="h5">Rowma Network Console</Typography>
            </Container>
          </Toolbar>
        </AppBar>
        <Container>
          <Grid container spacing={3}>
            <Grid item xs={4}>
              <Paper className={classes.paper}>
                <div>
                  <FormControl component="fieldset">
                    <Typography variant='h5'>Select Your Robot{"'"}s UUID</Typography>
                    <RadioGroup aria-label="robots" name="robots" value={selectedRobot} onChange={handleRobotChange}>
                    {robotUuids && robotUuids.map(uuid => {
                      return (
                        <FormControlLabel value={uuid} control={<Radio />} label={uuid} />
                      )
                    })}
                    </RadioGroup>
                  </FormControl>
                </div>
                <div>
                  <Button variant="contained" color={connectButtonColor} onClick={handleConnectClicked}>
                    {connectButtonText}
                  </Button>
                </div>
              </Paper>
            </Grid>

            <Grid item xs={4}>
              <Paper className={classes.paper}>
                <div>
                  <FormControl component="fieldset">
                    <Typography variant='h5'>Select a rosrun command</Typography>
                    <RadioGroup aria-label="rosrun" name="rosrun" value={selectedRosrunCommand} onChange={handleRosrunCommandChange}>
                    {rosrunCommands && rosrunCommands.map(command => {
                      return (
                        <FormControlLabel value={command} control={<Radio />} label={command} />
                      )
                    })}
                    </RadioGroup>
                  </FormControl>
                </div>
                <div>
                  <Button variant="contained" color="primary" onClick={handleRosrunButtonClick}>
                    Execute
                  </Button>
                </div>
              </Paper>
            </Grid>

            <Grid item xs={4}>
              <Paper className={classes.paper}>
                <div>
                  <FormControl component="fieldset">
                    <Typography variant='h5'>Select a roslaunch command</Typography>
                    <RadioGroup aria-label="roslaunch" name="roslaunch" value={selectedRoslaunchCommand} onChange={handleRoslaunchCommandChange}>
                    {roslaunchCommands && roslaunchCommands.map(command => {
                      return (
                        <FormControlLabel value={command} control={<Radio />} label={command} />
                      )
                    })}
                    </RadioGroup>
                  </FormControl>
                </div>
                <div>
                  <Button variant="contained" color="primary" onClick={handleRoslaunchButtonClick}>
                    Execute
                  </Button>
                </div>
              </Paper>
            </Grid>

            <Grid item xs={12}>
              <Paper className={classes.paper}>
                <div>
                  <p>Send (Topic Selectbox) from (Robot Selectbox) to (Robot Selectbox)</p>
                </div>
              </Paper>
            </Grid>

          </Grid>
        </Container>
      </ThemeProvider>
    </div>
  );
}

export default App
