"""Helper functions and classes."""

# %%
from dataclasses import dataclass
from datetime import datetime
import numpy as np
import os
from scipy.io import savemat
from types import SimpleNamespace
import pandas as pd


# %%
def abc2complex(u):
    """
    Transform three-phase quantities to a complex space vector.

    Parameters
    ----------
    u : array_like, shape (3,)
        Phase quantities.

    Returns
    -------
    complex
        Complex space vector (peak-value scaling).

    Examples
    --------
    >>> from motulator import abc2complex
    >>> y = abc2complex([1, 2, 3])
    >>> y
    (-1-0.5773502691896258j)

    """
    return (2/3)*u[0] - (u[1] + u[2])/3 + 1j*(u[1] - u[2])/np.sqrt(3)


# %%
def complex2abc(u):
    """
    Transform a complex space vector to three-phase quantities.

    Parameters
    ----------
    u : complex
        Complex space vector (peak-value scaling).

    Returns
    -------
    ndarray, shape (3,)
        Phase quantities.

    Examples
    --------
    >>> from motulator import complex2abc
    >>> y = complex2abc(1-.5j)
    >>> y
    array([ 1.       , -0.9330127, -0.0669873])

    """
    return np.array([
        u.real, .5*(-u.real + np.sqrt(3)*u.imag),
        .5*(-u.real - np.sqrt(3)*u.imag)
    ])


# %%
class Sequence:
    """
    Sequence generator.

    The time array must be increasing. The output values are interpolated
    between the data points.

    Parameters
    ----------
    times : ndarray
        Time values.
    values : ndarray
        Output values.
    periodic : bool, optional
        Enables periodicity. The default is False.

    """

    def __init__(self, times, values, periodic=False):
        self.times = times
        self.values = values
        if periodic is True:
            self._period = times[-1] - times[0]
        else:
            self._period = None

    def __call__(self, t):
        """
        Interpolate the output.

        Parameters
        ----------
        t : float
            Time.

        Returns
        -------
        float or complex
            Interpolated output.

        """
        return np.interp(t, self.times, self.values, period=self._period)


# %%
class Step:
    """Step function."""

    def __init__(self, step_time, step_value, initial_value=0):
        self.step_time = step_time
        self.step_value = step_value
        self.initial_value = initial_value

    def __call__(self, t):
        """
        Step function.

        Parameters
        ----------
        t : float
            Time.

        Returns
        -------
        float
            Step output.

        """
        return self.initial_value + (t >= self.step_time)*self.step_value


# %%
def wrap(theta):
    """
    Limit the angle into the range [-pi, pi).

    Parameters
    ----------
    theta : float
        Angle (rad).

    Returns
    -------
    float
        Limited angle.

    """

    return np.mod(theta + np.pi, 2*np.pi) - np.pi


# %%
@dataclass
class NominalValues:
    """
    Nominal values.

    Parameters
    ----------
    U : float
        Voltage (V, rms, line-line).
    I : float
        Current (A, rms).
    f : float
        Frequency (Hz).
    P : float
        Power (W).
    tau : float, optional
        Torque (Nm). The default value is None.

    """

    U: float
    I: float
    f: float
    P: float
    tau: float = None


# %%
@dataclass
class BaseValues:
    # pylint: disable=too-many-instance-attributes
    """
    Base values.

    Parameters
    ----------
    u : float
        Voltage (V, peak, line-neutral).
    i : float
        Current (A, peak).
    w : float
        Angular frequency (rad/s).
    psi : float
        Flux linkage (Vs).
    p : float
        Power (W).
    Z : float
        Impedance (Ω).
    L : float
        Inductance (H).
    C : float
        Capacitance (F).
    tau : float, optional
        Torque (Nm). The default is None.
    n_p : int, optional
        Number of pole pairs. The default is None.

    """
    u: float
    i: float
    w: float
    psi: float
    p: float
    Z: float
    L: float
    C: float
    tau: float = None
    n_p: int = None

    @classmethod
    def from_nominal(cls, nom, n_p=None):
        """
        Compute base values from nominal values.

        Parameters
        ----------
        nom : NominalValues
            Nominal values containing the following fields:

                U : float
                    Voltage (V, rms, line-line).
                I : float
                    Current (A, rms).
                f : float
                    Frequency (Hz).

        n_p : int, optional
            Number of pole pairs. If not given it is assumed that base values
            for a grid converter are calculated. The default is None.

        Returns
        -------
        BaseValues
            Base values.

        Notes
        -----
        Notice that the nominal torque is larger than the base torque due to
        the power factor and efficiency being less than unity.

        """
        u = np.sqrt(2/3)*nom.U
        i = np.sqrt(2)*nom.I
        w = 2*np.pi*nom.f
        psi = u/w
        p = 1.5*u*i
        Z = u/i
        L = Z/w
        C = 1/(Z*w)

        if n_p is not None:
            tau = n_p*p/w
            return cls(
                u=u, i=i, w=w, psi=psi, p=p, Z=Z, L=L, C=C, tau=tau, n_p=n_p)
        return cls(u=u, i=i, w=w, psi=psi, p=p, Z=Z, L=L, C=C)


# %%
class DataExporter:
    @staticmethod
    def namespace_to_dict(obj):
        """
        Convert a SimpleNamespace object or any object with __dict__ to a dictionary,
        filtering out None values and private attributes
        
        Parameters:
            obj: SimpleNamespace object or any object with attributes
        Returns:
            dict: Dictionary containing the object's valid attributes
        """
       
        if hasattr(obj, '__dict__'):
            return {key: value for key, value in obj.__dict__.items() 
                    if not key.startswith('_') and value is not None}
        return {}

    def _flatten_dict(self, d, parent_key='', sep='_'):
        """
        Flatten a nested dictionary structure into a single-level dictionary
        
        Args:
            d: dict to flatten
            parent_key: string to prepend to keys
            sep: separator between nested keys
        Returns:
            dict: Flattened dictionary
        """
        items = []
        for k, v in d.items():
            new_key = f"{parent_key}{sep}{k}" if parent_key else k
            if isinstance(v, dict):
                items.extend(self._flatten_dict(v, new_key, sep=sep).items())
            else:
                items.append((new_key, v))
        return dict(items)

    def save_mat(self, filename=None):
        """
        Export simulation data to MATLAB format, organizing mdl data by subsystems
        
        Parameters:
            filename: optional, str
            Output .mat filename
        """  
        def add_to_matlab_data(attr, name, target_dict):
            if hasattr(attr, '__dict__'):
                attr_dict = self.namespace_to_dict(attr)
                if attr_dict:
                    target_dict[name] = attr_dict

        ctrl = self.ctrl.data
        mdl = self.mdl
        matlab_data = {}
        timestamp = datetime.now().strftime('%Y%m%d_%H:%M_')

        # Export control data
        add_to_matlab_data(ctrl.ref, 'ctrl_ref', matlab_data)
        add_to_matlab_data(ctrl.fbk, 'ctrl_fbk', matlab_data)

        # Export model data organized by subsystems
        mdl_dict = {}
        for subsys_name in dir(mdl):
            if not subsys_name.startswith('_'):
                subsys = getattr(mdl, subsys_name)
                subsys_dict = {}
                if hasattr(subsys, 'data'):
                    add_to_matlab_data(subsys.data, 'data', subsys_dict)
                if hasattr(subsys, 'par'):
                    add_to_matlab_data(subsys.par, 'par', subsys_dict)
                if subsys_dict:
                    mdl_dict[subsys_name] = subsys_dict

        if mdl_dict:
            matlab_data['mdl'] = mdl_dict

        try:
            savemat(timestamp + filename + '.mat', matlab_data)
            print(f"Data successfully exported to {timestamp + filename}")
        except Exception as e:
            print(f"Error saving data: {str(e)}")
            for key, value in matlab_data.items():
                if value is None:
                    print(f"Found None value for key: {key}")

        return matlab_data

    def save_csv(self, base_filename=None, separate_files=True):
        """
        Export simulation data to CSV format
        
        Parameters:
            base_filename: str, base name for the CSV file(s)
            separate_files: bool, if True, creates separate CSV files for different data types
                          if False, creates a single CSV with flattened structure
        """
        timestamp = datetime.now().strftime('%Y%m%d_%H%M_')
        matlab_data = self.save_mat(filename=None)  # Get the structured data without saving
        
        if separate_files:
            # Create a directory to store CSV files if it doesn't exist
            directory = f"{timestamp+base_filename}_csv"
            os.makedirs(directory, exist_ok=True)
            
            # Save different components to separate CSV files
            for main_key, data in matlab_data.items():
                if isinstance(data, dict):
                    flat_data = self._flatten_dict(data)
                    # Ensure all values in flat_data are iterables
                    for key, value in flat_data.items():
                        if not isinstance(value, (list, np.ndarray)):
                            flat_data[key] = [value]
                    df = pd.DataFrame.from_dict(flat_data, orient='index').T
                    
                    # Save to CSV
                    filename = os.path.join(directory, f"{main_key}.csv")
                    df.to_csv(filename, index=False)
                    print(f"Data exported to {filename}")
        
        else:
            # Flatten the entire structure into a single dictionary
            flat_data = self._flatten_dict(matlab_data)

            # Ensure all values in flat_data are iterables
            for key, value in flat_data.items():
                if not isinstance(value, (list, np.ndarray)):
                    flat_data[key] = [value]
            
            # Convert to DataFrame
            df = pd.DataFrame.from_dict(flat_data, orient='index').T
            
            # Save to single CSV
            filename = f"{timestamp+base_filename}.csv"
            df.to_csv(filename, index=False)
            print(f"Data exported to {filename}")