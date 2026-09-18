import { CommonModule, DecimalPipe } from '@angular/common';
import { Component, OnDestroy, OnInit } from '@angular/core';
import { Subscription, interval } from 'rxjs';
import { MonitoringApiService } from './monitoring-api.service';
import { MapMetadata, VehicleStatus, VehiclesResponse } from './models';

@Component({
  selector: 'app-root',
  standalone: true,
  imports: [CommonModule, DecimalPipe],
  templateUrl: './app.component.html',
  styleUrl: './app.component.css',
})
export class AppComponent implements OnInit, OnDestroy {
  vehicles: VehicleStatus[] = [];
  selectedVehicleId = '';
  map: MapMetadata | null = null;
  backendOnline = false;
  streamOnline = false;
  lastUpdated: Date | null = null;
  readonly mapImageUrl: string;
  detectionsFrameUrl: string | null = null;
  birdEyeFrameUrl: string | null = null;
  detectionsFrameLive = false;
  birdEyeFrameLive = false;

  private eventSource: EventSource | null = null;
  private pollSubscription: Subscription | null = null;
  private frameSubscription: Subscription | null = null;
  private frameRequests = { detections: false, 'bird-eye': false };

  constructor(private readonly api: MonitoringApiService) {
    this.mapImageUrl = api.mapImageUrl();
  }

  ngOnInit(): void {
    this.api.map().subscribe({
      next: value => this.map = value,
      error: () => this.map = null,
    });
    this.refresh();
    this.eventSource = this.api.events(
      response => {
        this.streamOnline = true;
        this.applyVehicles(response);
      },
      () => this.streamOnline = false,
    );
    this.pollSubscription = interval(2000).subscribe(() => {
      if (!this.streamOnline) this.refresh();
    });
    this.refreshDebugFrames();
    this.frameSubscription = interval(250).subscribe(() => this.refreshDebugFrames());
  }

  ngOnDestroy(): void {
    this.eventSource?.close();
    this.pollSubscription?.unsubscribe();
    this.frameSubscription?.unsubscribe();
    this.revokeFrameUrl('detections');
    this.revokeFrameUrl('bird-eye');
  }

  get vehicle(): VehicleStatus | null {
    return this.vehicles.find(item => item.vehicle_id === this.selectedVehicleId)
      ?? this.vehicles[0]
      ?? null;
  }

  selectVehicle(event: Event): void {
    this.selectedVehicleId = (event.target as HTMLSelectElement).value;
  }

  refresh(): void {
    this.api.vehicles().subscribe({
      next: response => this.applyVehicles(response),
      error: () => this.backendOnline = false,
    });
  }

  private refreshDebugFrames(): void {
    this.refreshDebugFrame('detections');
    this.refreshDebugFrame('bird-eye');
  }

  private refreshDebugFrame(kind: 'bird-eye' | 'detections'): void {
    if (this.frameRequests[kind]) return;
    this.frameRequests[kind] = true;
    this.api.debugFrame(kind).subscribe({
      next: blob => {
        this.frameRequests[kind] = false;
        const nextUrl = URL.createObjectURL(blob);
        this.revokeFrameUrl(kind);
        if (kind === 'detections') {
          this.detectionsFrameUrl = nextUrl;
          this.detectionsFrameLive = true;
        } else {
          this.birdEyeFrameUrl = nextUrl;
          this.birdEyeFrameLive = true;
        }
      },
      error: () => {
        this.frameRequests[kind] = false;
        if (kind === 'detections') this.detectionsFrameLive = false;
        else this.birdEyeFrameLive = false;
      },
    });
  }

  private revokeFrameUrl(kind: 'bird-eye' | 'detections'): void {
    const value = kind === 'detections'
      ? this.detectionsFrameUrl
      : this.birdEyeFrameUrl;
    if (value) URL.revokeObjectURL(value);
    if (kind === 'detections') this.detectionsFrameUrl = null;
    else this.birdEyeFrameUrl = null;
  }

  poseMarkerStyle(): Record<string, string> {
    const pose = this.vehicle?.pose?.pose;
    if (!pose || !this.map?.width_x_m || !this.map.height_y_m) {
      return { display: 'none' };
    }
    const left = Math.max(0, Math.min(100, pose.x_m / this.map.width_x_m * 100));
    const top = Math.max(0, Math.min(100, (1 - pose.y_m / this.map.height_y_m) * 100));
    const degrees = -pose.yaw_rad * 180 / Math.PI;
    return {
      left: `${left}%`,
      top: `${top}%`,
      transform: `translate(-50%, -50%) rotate(${degrees}deg)`,
    };
  }

  clockwiseDegrees(mapRadians: number | undefined): number | null {
    return mapRadians === undefined ? null : -mapRadians * 180 / Math.PI;
  }

  slotState(slotId: string): string {
    return this.vehicle?.parking?.slots?.find(slot => slot.id === slotId)?.state ?? 'UNKNOWN';
  }

  slotConfidence(slotId: string): number | null {
    return this.vehicle?.parking?.slots?.find(slot => slot.id === slotId)?.confidence ?? null;
  }

  parkingSignConfidence(): number | null {
    const signs = this.vehicle?.parking?.objects
      ?.filter(object => object.class === 'parking_sign') ?? [];
    return signs.length
      ? Math.max(...signs.map(object => object.confidence))
      : null;
  }

  detectedVehicleCount(): number {
    return this.vehicle?.parking?.objects
      ?.filter(object => object.class === 'vehicle').length ?? 0;
  }

  associatedVehicleSlots(): string {
    const slots = this.vehicle?.parking?.objects
      ?.filter(object => object.class === 'vehicle' && object.associated_slot)
      .map(object => object.associated_slot!) ?? [];
    return [...new Set(slots)].join(', ') || '—';
  }

  trajectoryValue(key: string): unknown {
    return this.vehicle?.trajectory?.[key] ?? null;
  }

  localTrajectoryPolyline(): string | null {
    const points = this.vehicle?.local_trajectory?.points;
    if (!points?.length || !this.map?.width_x_m || !this.map.height_y_m) {
      return null;
    }
    return points.map(point => {
      const x = Math.max(0, Math.min(1000,
        point.x_m / this.map!.width_x_m * 1000));
      const y = Math.max(0, Math.min(1000,
        (1 - point.y_m / this.map!.height_y_m) * 1000));
      return `${x},${y}`;
    }).join(' ');
  }

  safetyValue(key: string): unknown {
    return this.vehicle?.safety?.[key] ?? null;
  }

  shortHash(value: string | undefined): string {
    return value ? `${value.slice(0, 8)}…${value.slice(-6)}` : '—';
  }

  private applyVehicles(response: VehiclesResponse): void {
    this.vehicles = response.vehicles;
    if (!this.selectedVehicleId && this.vehicles.length) {
      this.selectedVehicleId = this.vehicles[0].vehicle_id;
    }
    if (this.selectedVehicleId && !this.vehicles.some(v => v.vehicle_id === this.selectedVehicleId)) {
      this.selectedVehicleId = this.vehicles[0]?.vehicle_id ?? '';
    }
    this.backendOnline = true;
    this.lastUpdated = new Date();
  }
}
