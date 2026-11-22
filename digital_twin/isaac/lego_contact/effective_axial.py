"""Passive axial force foundation; no joint, attachment, capture or success logic.

Depth increases inward. Positive force acts outward on the upper reference
point. U(depth)=integral elastic_force ds, F=elastic_force+c*depth_rate,
and connector power is -F*depth_rate. Thus P+dU/dt=-c*depth_rate**2.
"""
import json
from pathlib import Path
import numpy as np


class ModelDomainError(ValueError):
    """The supplied coordinate or admission state is outside the model domain."""


class EffectiveAxialModel:
    """Evaluate explicitly supplied lumped force coefficients in SI units."""

    def __init__(self, config):
        self.config = config
        self.depth = np.asarray(config.get("depth_knots_m"), dtype=float)
        self.elastic = np.asarray(config.get("elastic_force_N"), dtype=float)
        self.damping = np.asarray(config.get("damping_N_s_m"), dtype=float)
        if any(a.ndim != 1 for a in (self.depth,self.elastic,self.damping)):
            raise ValueError("identified curve arrays are required; null parameters cannot run")
        if len(self.depth)<2 or self.elastic.shape!=self.depth.shape or self.damping.shape!=self.depth.shape:
            raise ValueError("equal-length curve arrays with at least two knots required")
        if not all(np.isfinite(a).all() for a in (self.depth,self.elastic,self.damping)):
            raise ValueError("finite curve coefficients required")
        if self.depth[0]!=0 or self.elastic[0]!=0 or self.damping[0]!=0:
            raise ValueError("depth, elastic force and damping must start at zero contact")
        if np.any(np.diff(self.depth)<=0) or np.any(self.damping<0):
            raise ValueError("depths must increase and damping must be nonnegative")
        if config.get("real_ready",False):
            raise ValueError("this foundation does not admit real-ready configurations")
        self.slopes=np.diff(self.elastic)/np.diff(self.depth)
        self.energy=np.r_[0.,np.cumsum(np.diff(self.depth)*(self.elastic[:-1]+self.elastic[1:])/2)]
        minimum=float(self.energy.min())
        for i,slope in enumerate(self.slopes):
            if slope!=0:
                offset=-self.elastic[i]/slope
                if 0<offset<self.depth[i+1]-self.depth[i]:
                    minimum=min(minimum,float(self.energy[i]+self.elastic[i]*offset+.5*slope*offset**2))
        if minimum < -1e-12:
            raise ValueError("elastic potential below the separated zero-energy state")

    @classmethod
    def from_file(cls,path):
        """Load curve coefficients from a JSON file; no external state is restored."""
        return cls(json.loads(Path(path).read_text()))

    def evaluate(self,depth_m,depth_rate_m_s):
        """Return outward force, potential, dissipation and connector power.

        Args: depth_m and depth_rate_m_s are finite scalar SI coordinates.
        Returns: dictionary of SI scalar values; no seating/attachment success.
        Raises: ModelDomainError past the identified depth domain.
        """
        depth=float(depth_m);speed=float(depth_rate_m_s)
        if not np.isfinite([depth,speed]).all():
            raise ModelDomainError("finite axial coordinate and rate required")
        if depth<=0:
            return {"force_N":0.,"stored_energy_J":0.,"dissipation_W":0.,"connector_power_W":0.}
        if depth>self.depth[-1]:
            raise ModelDomainError("depth exceeds supplied model domain; no invented hard stop")
        i=min(int(np.searchsorted(self.depth,depth,side="right")-1),len(self.depth)-2)
        offset=depth-self.depth[i]
        elastic=float(self.elastic[i]+self.slopes[i]*offset)
        damping=float(np.interp(depth,self.depth,self.damping))
        energy=float(self.energy[i]+self.elastic[i]*offset+.5*self.slopes[i]*offset**2)
        force=elastic+damping*speed
        return {"force_N":force,"stored_energy_J":max(0.,energy),
                "dissipation_W":damping*speed*speed,"connector_power_W":-force*speed}

    def check_admission(self,xy_error_m,yaw_error_rad,tilt_rad):
        """Reject unsupported lateral/rotational states; supplies no capture force."""
        limits=self.config.get("admission",{})
        values={"xy_m":xy_error_m,"yaw_rad":yaw_error_rad,"tilt_rad":tilt_rad}
        for key,value in values.items():
            limit=limits.get(key)
            if limit is None or not np.isfinite([value,limit]).all() or limit<0 or abs(value)>limit:
                raise ModelDomainError("unsupported axial admission: "+key)
