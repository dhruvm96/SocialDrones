#!/usr/bin/perl
use warnings;
use strict;

use constant { TRUE => 1, FALSE => 0 };
use JSON;
my $json = JSON->new->allow_nonref;

my @files = glob("*.log");
foreach my $filename (@files)
{
	open my $in, "< $filename";
	my $commands = {};
	my $first_pwm = TRUE;
	my $first_timestamp = 0;
	my $last_time = 0;
	my $csv = "time, m1_pwm, m2_pwm, m3_pwm, m4_pwm, bat_volt, power_m1, power_m2, power_m3, power_m4, power_total\n";
	foreach my $line (<$in>)
	{
		if ($line =~ /^###Command: (.*)###$/)
		{
			$commands->{$last_time} = $1;
		}
		elsif ($line =~ /^\[(\d+)\]\[PWM\]:(.*)$/)
		{
			if ($first_pwm)
			{
				$first_timestamp = $1;
				$first_pwm = FALSE;
			}
			my $time = $1 - $first_timestamp;
			$last_time = $time;
			(my $data = $2) =~ s/'/"/g;
			$data = $json->decode( $data );
			my ($A, $B, $C, $D) = (-1.77338*(10**-15), 1.05305*(10**-9), -7.92635*(10**-6 ), 0.0248703);
			my $power_m1 = $A*$data->{'pwm.m1_pwm'}**3 +
						$B*$data->{'pwm.m1_pwm'}**2 +
						$C*$data->{'pwm.m1_pwm'} + 
						$D;
			my $power_m2 = $A*$data->{'pwm.m2_pwm'}**3 +
						$B*$data->{'pwm.m2_pwm'}**2 +
						$C*$data->{'pwm.m2_pwm'} +
						$D;
			my $power_m3 = $A*$data->{'pwm.m3_pwm'}**3 +
						$B*$data->{'pwm.m3_pwm'}**2 +
						$C*$data->{'pwm.m3_pwm'} +
						$D;
			my $power_m4 = $A*$data->{'pwm.m4_pwm'}**3 +
						$B*$data->{'pwm.m4_pwm'}**2 +
						$C*$data->{'pwm.m4_pwm'} +
						$D;
			my $power_total = $power_m1 + $power_m2 + $power_m3 + $power_m4;
			my $csv_line = "$time, $data->{'pwm.m1_pwm'}, 
							$data->{'pwm.m2_pwm'}, $data->{'pwm.m3_pwm'}, 
							$data->{'pwm.m4_pwm'}, $data->{'pm.vbat'}, 
						    $power_m1, $power_m2, $power_m3, 
						    $power_m4, $power_total\n";
			$csv .= $csv_line;
		}
	}
	close $in;
			
	(my $title = $filename) =~ s/\.log//;
	open my $out, "> $title.csv";
	print $out $csv;
	close $out;
	open $out, "> $title.commands";
	foreach my $key (sort {$a <=> $b} keys(%$commands))
	{
		print $out "$key, $commands->{$key}\n";
	}
	close $out;
}

	
